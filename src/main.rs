#![no_std]
#![no_main]

mod core;
mod prelude;

use core::OutputSpiDevice;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    // delay::Delay,
    dma::*,
    dma_buffers,
    gpio::{GpioPin, Io, Level, Output, NO_PIN},
    peripherals::Peripherals,
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{Spi, SpiDmaBus},
        FullDuplexMode, SpiMode,
    },
    system::SystemControl,
    Async,
};
use prelude::*;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

type Spi2Bus = Mutex<NoopRawMutex, SpiDmaBus<'static, SPI2, DmaChannel0, FullDuplexMode, Async>>;

#[embassy_executor::task]
async fn blink_led(mut led: Output<'static, GpioPin<4>>) {
    led.set_high();
    loop {
        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
    }
}

async fn generic_run_strip(device: (impl OutputSpiDevice + 'static), diodes: u8, speed: u16) {
    let mut andon_light = core::AndonLight::new(diodes, speed);
    andon_light.run_test_procedure(device).await;
}

#[embassy_executor::task]
async fn run_strip(device: (impl OutputSpiDevice + 'static), diodes: u8, speed: u16) {
    generic_run_strip(device, diodes, speed).await;
}

#[embassy_executor::task]
async fn run_strip2(device: (impl OutputSpiDevice + 'static), diodes: u8, speed: u16) {
    generic_run_strip(device, diodes, speed).await;
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    // let delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    // let _init = esp_wifi::initialize(
    //     esp_wifi::EspWifiInitFor::Wifi,
    //     timg0.timer0,
    //     esp_hal::rng::Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    //     &clocks,
    // )
    // .unwrap();

    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = Output::new(io.pins.gpio4, Level::High);
    spawner.spawn(blink_led(led)).unwrap();

    let sclk = io.pins.gpio8;
    let miso = io.pins.gpio9;
    let mosi = io.pins.gpio10;
    // TODO make it actually static
    let cs = ControlPin {
        pin: Output::new(io.pins.gpio2, Level::High),
    };
    let cs2 = ControlPin {
        pin: Output::new(io.pins.gpio3, Level::High),
    };

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi: SpiDmaBus<SPI2, DmaChannel0, FullDuplexMode, Async> =
        Spi::new(peripherals.SPI2, 3u32.MHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN)
            .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

    static SPI_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    let spi_dev = SpiDev {
        device: SpiDevice::new(spi_bus, cs),
    };
    spawner.spawn(run_strip(spi_dev, 16, 150)).unwrap();
    Timer::after(Duration::from_millis(77)).await;
    let spi_dev = SpiDev {
        device: SpiDevice::new(spi_bus, cs2),
    };
    spawner.spawn(run_strip2(spi_dev, 4, 300)).unwrap();
}

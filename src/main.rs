#![no_std]
#![no_main]

mod core;
mod prelude;

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

use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_hal::timer::timg::TimerGroup;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn blink_led(mut led: Output<'static, GpioPin<4>>) {
    led.set_high();
    loop {
        Timer::after(Duration::from_millis(200)).await;
        led.toggle();
    }
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
    let cs: ControlPin<'static, GpioPin<2>> = ControlPin {
        pin: Output::new(io.pins.gpio2, Level::High),
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

    static SPI_BUS: StaticCell<core::Spi2Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    let mut andon_light = core::AndonLight::new(16);
    andon_light.run_test_procedure(spi_bus, cs).await;
}

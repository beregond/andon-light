#![no_std]
#![no_main]

use andon_light::*;

use andon_light_core::ErrorCodesBase;
use andon_light_macros::{generate_default_from_env, ErrorCodesEnum};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use static_cell::StaticCell;

extern crate alloc;

type SpiBus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;

#[derive(Debug, Hash, Eq, PartialEq, ErrorCodesEnum)]
pub enum ErrorCodes {
    #[code(message = "Ok", level = "ok")]
    Ok,
    #[code(message = "Device is idle", level = "idle")]
    I001,
    #[code(message = "Device reports warning", level = "warn")]
    W001,
    #[code(message = "TCS sensor is unavailable", level = "system_error")]
    SE001,
    #[code(message = "Failed to enable TCS sensor", level = "system_error")]
    SE002,
    #[code(message = "TCS sensor error", level = "system_error")]
    SE003,
    #[code(message = "Device report error", level = "error")]
    E001,
}

type AndonLight = andon_light_core::AndonLight<ErrorCodes, { ErrorCodes::MIN_SET_SIZE }>;
type AndonAsyncMutex = Mutex<CriticalSectionRawMutex, AndonLight>;

#[embassy_executor::task]
async fn run_andon_light(
    mut device: SpiDev<'static>,
    andon_light: &'static AndonAsyncMutex,
    mut led_reset: Output<'static>,
) {
    log::debug!("Resetting leds");
    led_reset.set_high();
    led_reset.set_low();

    {
        log::debug!("Start of test procedure");
        let mut andon_light = andon_light.lock().await;

        andon_light.run_test_procedure(&mut device).await;
        andon_light.signal(&mut device).await;
        Timer::after(Duration::from_millis(andon_light.get_speed() as u64)).await;
        log::debug!("End of test procedure");
    }

    log::debug!("Entering regular cycle");
    loop {
        let pause: u64;
        {
            let mut andon_light = andon_light.lock().await;
            andon_light.signal(&mut device).await;
            pause = andon_light.get_speed() as u64;
        }
        Timer::after(Duration::from_millis(pause)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1
    esp_println::logger::init_logger_from_env();
    log::info!("Hello world!");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    log::info!("Embassy initialized!");

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // Define Spi
    let sclk = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let miso = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::None),
    );
    let mosi = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(4))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    log::debug!("Peripherals initialized");
    log::debug!("Reading config from SD card");
    // SD card read be here
    log::debug!("End of config read");

    log::debug!("Starting up leds control");

    let leds_select = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let spi_dev = SpiDev {
        device: SpiDevice::new(spi_bus, leds_select),
    };

    let led_reset = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    static ANDON: StaticCell<AndonAsyncMutex> = StaticCell::new();
    let andon_light = ANDON.init(Mutex::new(AndonLight::new(16, 10, 150)));
    spawner
        .spawn(run_andon_light(spi_dev, andon_light, led_reset))
        .unwrap();

    log::debug!("Leds started");

    loop {
        log::info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

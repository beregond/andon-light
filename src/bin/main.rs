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
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use esp_hal::{gpio::GpioPin, timer::systimer::SystemTimer};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

extern crate alloc;

const CONFIG_BUFFER_SIZE: usize = 4096;
const DEFAULT_LED_AMOUNT: usize = generate_default_from_env!(DEFAULT_LED_AMOUNT, 16);

const fn _default_leds() -> usize {
    DEFAULT_LED_AMOUNT
}

const fn _default_brightness() -> u8 {
    10
}

const fn _default_speed() -> u16 {
    150
}

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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AndonConfig {
    version: u32,
    core_config: CoreConfig,
    // buzzer_enabled: bool,
}

impl Default for AndonConfig {
    fn default() -> Self {
        Self {
            version: 1,
            core_config: CoreConfig::default(),
            // buzzer_enabled: true,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CoreConfig {
    #[serde(default = "_default_leds")]
    leds_amount: usize,
    #[serde(default = "_default_brightness")]
    brightness: u8,
    #[serde(default = "_default_speed")]
    speed: u16,
}

impl Default for CoreConfig {
    fn default() -> Self {
        serde_json_core::de::from_slice(b"{}").unwrap().0
    }
}

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

#[embassy_executor::task]
async fn heartbeat(
    mut lstimer: timer::Timer<'static, LowSpeed>,
    mut channel: channel::Channel<'static, LowSpeed>,
) {
    log::debug!("Heartbeat task started");
    lstimer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    channel
        .configure(channel::config::Config {
            timer: &lstimer,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    loop {
        channel.start_duty_fade(30, 90, 100).unwrap();
        Timer::after(Duration::from_millis(100)).await;
        channel.start_duty_fade(90, 10, 100).unwrap();
        Timer::after(Duration::from_millis(100)).await;
        channel.start_duty_fade(10, 30, 100).unwrap();
        Timer::after(Duration::from_millis(800)).await;
    }
}

#[embassy_executor::task]
async fn buzzer(mut buzzer_pin: GpioPin<3>, ledc: Ledc<'static>) {
    let tones: [(char, u32); 4] = [
        ('d', 294),
        ('f', 349),
        ('a', 440),
        ('o', 587), // d octave
    ];
    let tune = [('d', 1), ('f', 1), ('a', 1), ('o', 1), (' ', 2)];
    // Obtain a note in the tune
    for note in tune {
        // Retrieve the freqeuncy and beat associated with the note
        for tone in tones {
            // Find a note match in the tones array and update
            if tone.0 == note.0 {
                log::debug!("Note: {}", note.0);
                let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer1);
                lstimer
                    .configure(timer::config::Config {
                        duty: timer::config::Duty::Duty13Bit,
                        clock_source: timer::LSClockSource::APBClk,
                        frequency: Rate::from_hz(tone.1),
                    })
                    .unwrap();

                let mut channel = ledc.channel(channel::Number::Channel1, &mut buzzer_pin);
                channel
                    .configure(channel::config::Config {
                        timer: &lstimer,
                        duty_pct: 10,
                        pin_config: channel::config::PinConfig::PushPull,
                    })
                    .unwrap();
                Timer::after(Duration::from_millis(100 * note.1)).await;
            } else if note.0 == ' ' {
                log::debug!("Empty note");
                let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer1);
                lstimer
                    .configure(timer::config::Config {
                        duty: timer::config::Duty::Duty13Bit,
                        clock_source: timer::LSClockSource::APBClk,
                        frequency: Rate::from_hz(1_u32),
                    })
                    .unwrap();

                let mut channel = ledc.channel(channel::Number::Channel1, &mut buzzer_pin);
                channel
                    .configure(channel::config::Config {
                        timer: &lstimer,
                        duty_pct: 0,
                        pin_config: channel::config::PinConfig::PushPull,
                    })
                    .unwrap();
                Timer::after(Duration::from_millis(100 * note.1)).await;
            }
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
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

    // Configure the LEDC
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Heartbeat
    let heartbeat_led = peripherals.GPIO4;
    let lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let channel0 = ledc.channel(channel::Number::Channel0, heartbeat_led);
    spawner.spawn(heartbeat(lstimer0, channel0)).unwrap();

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

    let mut buffer = [0u8; CONFIG_BUFFER_SIZE];
    let sd_select = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    let sd_reader = SdReader::new(spi_bus, sd_select, Delay::new());
    // TODO: Need better support for config errors
    let result = sd_reader
        .read_config::<AndonConfig>("CONFIG.JSO", &mut buffer)
        .await;

    let andon_config = match result {
        Ok(maybe_result) => {
            if let Some(config) = maybe_result {
                log::debug!("Config read properly from file");
                config
            } else {
                log::debug!("Seems there is no config file, falling back to default");
                AndonConfig::default()
            }
        }
        Err(e) => {
            log::error!("Failed to read config file");
            log::error!("{:?}", e);
            AndonConfig::default()
        }
    };

    log::debug!("End of config read");

    // Buzzer
    let buzzer_pin = peripherals.GPIO3;
    spawner.spawn(buzzer(buzzer_pin, ledc)).unwrap();

    log::debug!("Starting up leds control");

    let leds_select = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let spi_dev = SpiDev {
        device: SpiDevice::new(spi_bus, leds_select),
    };

    let led_reset = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    static ANDON: StaticCell<AndonAsyncMutex> = StaticCell::new();
    let andon_light = ANDON.init(Mutex::new(AndonLight::new(
        andon_config.core_config.leds_amount as u8,
        andon_config.core_config.brightness,
        andon_config.core_config.speed,
    )));
    spawner
        .spawn(run_andon_light(spi_dev, andon_light, led_reset))
        .unwrap();

    log::debug!("Leds started");
}

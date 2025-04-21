#![no_std]
#![no_main]

use andon_light::*;

use andon_light_core::{AlertLevel, ErrorCodesBase};
use andon_light_macros::{generate_default_from_env, ErrorCodesEnum};
use core::cmp::Ord;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::I2c,
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
use tcs3472::Tcs3472;
extern crate alloc;

const CONFIG_BUFFER_SIZE: usize = 4096;
const DEFAULT_LED_AMOUNT: usize = generate_default_from_env!(DEFAULT_LED_AMOUNT, 16);

#[inline]
const fn _default_leds() -> usize {
    DEFAULT_LED_AMOUNT
}

#[inline]
const fn _default_brightness() -> u8 {
    10
}

#[inline]
const fn _default_speed() -> u16 {
    150
}

#[inline]
const fn _default_true() -> bool {
    true
}

#[inline]
const fn _default_version() -> u32 {
    1
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
    #[serde(default = "_default_version")]
    version: u32,
    #[serde(default)]
    core_config: CoreConfig,
    #[serde(default = "_default_true")]
    buzzer_enabled: bool,
}

impl Default for AndonConfig {
    fn default() -> Self {
        serde_json_core::de::from_slice(b"{}").unwrap().0
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
        log::debug!("Start of led test procedure");
        let mut andon_light = andon_light.lock().await;

        andon_light.run_test_procedure(&mut device).await;
        andon_light.signal(&mut device).await;
        Timer::after(Duration::from_millis(andon_light.get_speed() as u64)).await;
        log::debug!("End of test procedure");
    }

    log::debug!("Entering regular leds cycle");
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
async fn buzzer(
    mut buzzer_pin: GpioPin<3>,
    ledc: Ledc<'static>,
    andon_light: &'static AndonAsyncMutex,
) {
    log::debug!("Buzzer enabled");
    let tones = [
        ("c4", 262),
        ("d4", 294),
        ("f4", 349),
        ("a4", 440),
        ("c5", 523),
        ("d5", 587),
        ("c6", 1046),
        (" ", 1),
    ];
    let mut tune = [("d4", 1), ("f4", 1), ("a4", 1), ("d5", 1), (" ", 2)];
    let mut first_run = true;
    let mut last_level = AlertLevel::Chill;
    loop {
        'melody: for note in tune {
            if !first_run {
                let andon_light = andon_light.lock().await;
                let alert_level = andon_light.calculate_alert_level();
                if last_level != alert_level {
                    match alert_level {
                        AlertLevel::Chill => {
                            tune = [(" ", 1), ("-", 0), ("-", 0), ("-", 0), ("-", 0)]
                        }
                        AlertLevel::Attentive => {
                            tune = [("c4", 1), (" ", 100), ("-", 0), ("-", 0), ("-", 0)]
                        }
                        AlertLevel::Important => {
                            tune = [("c5", 1), (" ", 10), ("-", 0), ("-", 0), ("-", 0)]
                        }
                        AlertLevel::Urgent => {
                            tune = [("c6", 1), (" ", 1), ("-", 0), ("-", 0), ("-", 0)]
                        }
                    }
                    last_level = alert_level;
                    break 'melody;
                }
            }
            if note.0 == "-" {
                continue;
            }
            // Obtain a note in the tune
            for tone in tones {
                // Find a note match in the tones array and update
                if tone.0 == note.0 {
                    log::trace!("Note: {}", note.0);
                    let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer1);
                    lstimer
                        .configure(timer::config::Config {
                            duty: timer::config::Duty::Duty13Bit,
                            clock_source: timer::LSClockSource::APBClk,
                            frequency: Rate::from_hz(tone.1),
                        })
                        .unwrap();

                    let duty_pct = match tone.0 {
                        " " => 0,
                        _ => 20,
                    };
                    let mut channel = ledc.channel(channel::Number::Channel1, &mut buzzer_pin);
                    channel
                        .configure(channel::config::Config {
                            timer: &lstimer,
                            duty_pct,
                            pin_config: channel::config::PinConfig::PushPull,
                        })
                        .unwrap();
                    for _ in 0..note.1 {
                        Timer::after(Duration::from_millis(100)).await;
                    }
                }
            }
        }
        if first_run {
            first_run = false;
            let andon_light = andon_light.lock().await;
            let alert_level = andon_light.calculate_alert_level();
            match alert_level {
                AlertLevel::Chill => tune = [(" ", 1), ("-", 0), ("-", 0), ("-", 0), ("-", 0)],
                AlertLevel::Attentive => {
                    tune = [("c4", 1), (" ", 100), ("-", 0), ("-", 0), ("-", 0)]
                }
                AlertLevel::Important => {
                    tune = [("c5", 1), (" ", 10), ("-", 0), ("-", 0), ("-", 0)]
                }
                AlertLevel::Urgent => tune = [("c6", 1), (" ", 1), ("-", 0), ("-", 0), ("-", 0)],
            }
            last_level = alert_level;
        }
    }
}

#[embassy_executor::task]
async fn rgb_probe_task(
    mut sensor: Tcs3472<I2c<'static, Async>>,
    andon_light: &'static AndonAsyncMutex,
) {
    log::debug!("RGB probe task started");
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        'main: loop {
            match (
                sensor.enable().await,
                andon_light.lock().await,
                ErrorCodes::SE001,
            ) {
                (Err(_), mut andon_light, code) => {
                    andon_light.notify(code);
                    break 'main;
                }
                (Ok(_), mut andon_light, code) => {
                    andon_light.resolve(code);
                }
            }

            match (
                sensor.enable_rgbc().await,
                andon_light.lock().await,
                ErrorCodes::SE002,
            ) {
                (Err(_), mut andon_light, code) => {
                    andon_light.notify(code);
                    break 'main;
                }
                (Ok(_), mut andon_light, code) => {
                    andon_light.resolve(code);
                }
            }

            while !sensor.is_rgbc_status_valid().await.unwrap() {
                // wait for measurement to be available
                Timer::after(Duration::from_millis(10)).await;
            }

            let mut exclusive = heapless::Vec::<_, 4>::new();
            exclusive.push(ErrorCodes::SE003).unwrap();
            exclusive.push(ErrorCodes::E001).unwrap();
            exclusive.push(ErrorCodes::I001).unwrap();
            exclusive.push(ErrorCodes::W001).unwrap();

            'inner: loop {
                match sensor.read_all_channels().await {
                    Err(_) => {
                        let mut andon_light = andon_light.lock().await;
                        andon_light.notify(ErrorCodes::SE003);
                        break 'inner;
                    }
                    Ok(measurement) => {
                        let color =
                            normalize_rgb(measurement.red, measurement.green, measurement.blue);
                        {
                            log::trace!("Color detected: {}", color);
                            let mut andon_light = andon_light.lock().await;
                            andon_light.resolve(ErrorCodes::SE003);
                            let error_code = match color {
                                "Green" | "Mint" | "Lime" => ErrorCodes::Ok,
                                "Blue" | "Violet" | "Azure" => ErrorCodes::I001,
                                "Red" | "Pink" | "Magenta" => ErrorCodes::E001,
                                _ => ErrorCodes::W001,
                            };
                            andon_light.notify_exclusive(error_code, &exclusive);
                        }
                    }
                }
                ticker.next().await;
            }
            ticker.next().await;
        }
        ticker.next().await;
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

    // Buzzer
    if andon_config.buzzer_enabled {
        let buzzer_pin = peripherals.GPIO3;
        spawner
            .spawn(buzzer(buzzer_pin, ledc, andon_light))
            .unwrap();
    } else {
        log::debug!("Buzzer disabled");
    }

    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let sensor = Tcs3472::new(i2c);
    spawner.spawn(rgb_probe_task(sensor, andon_light)).unwrap();
}

enum ColorLevel {
    High,
    Medium,
    Low,
}

// Function to determine the basic color based on normalized RGB values
fn map_to_basic_color(r: u16, g: u16, b: u16) -> &'static str {
    // Define the basic 12 colors. These are just names for simplicity.
    log::trace!("RGB values: {} {} {}", r, g, b);
    match map_three(r, g, b) {
        (ColorLevel::High, ColorLevel::Low, ColorLevel::High) => "Magenta",
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Medium) => "Pink",
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Low) => "Red",
        (ColorLevel::High, ColorLevel::Medium, ColorLevel::Low) => "Orange",
        (ColorLevel::High, ColorLevel::High, ColorLevel::Low) => "Yellow",
        (ColorLevel::Medium, ColorLevel::High, ColorLevel::Low) => "Lime",
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Low) => "Lime",
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Medium) => "Mint",
        (ColorLevel::Low, ColorLevel::High, ColorLevel::High) => "Cyan",
        (ColorLevel::Low, ColorLevel::Medium, ColorLevel::High) => "Azure",
        (ColorLevel::Low, ColorLevel::Low, ColorLevel::High) => "Blue",
        (ColorLevel::Medium, ColorLevel::Low, ColorLevel::High) => "Violet",
        (_, _, _) => "Gray",
    }
}

fn map_three(r: u16, g: u16, b: u16) -> (ColorLevel, ColorLevel, ColorLevel) {
    if r == 100 {
        let (g, b) = map_two(g, b);
        (ColorLevel::High, g, b)
    } else if g == 100 {
        let (r, b) = map_two(r, b);
        (r, ColorLevel::High, b)
    } else {
        let (r, g) = map_two(r, g);
        (r, g, ColorLevel::High)
    }
}

#[inline]
fn map_two(first: u16, second: u16) -> (ColorLevel, ColorLevel) {
    if first > second {
        (map_one(first), ColorLevel::Low)
    } else {
        (ColorLevel::Low, map_one(second))
    }
}

#[inline]
fn map_one(value: u16) -> ColorLevel {
    if value > 90 {
        ColorLevel::High
    } else if value > 60 {
        ColorLevel::Medium
    } else {
        ColorLevel::Low
    }
}

fn normalize_rgb(red: u16, green: u16, blue: u16) -> &'static str {
    let max = max_of_three(red, green, blue);
    if max == 0 {
        return "Black";
    }
    let normalize = |val: u16| (val as f32 / max as f32 * 100.0) as u16;
    let (r, g, b) = (normalize(red), normalize(green), normalize(blue));
    map_to_basic_color(r, g, b)
}

fn max_of_three<T: Ord>(a: T, b: T, c: T) -> T {
    a.max(b).max(c)
}

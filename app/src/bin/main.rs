#![no_std]
#![no_main]

use andon_light::*;

use andon_light_core::{
    colors::{Color, ColorMapper},
    AlertLevel, ErrorCodesBase,
};
use andon_light_macros::{default_from_env, ErrorCodesEnum};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::spi::SpiDevice as _;
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
use esp_wifi::wifi;
use heapless::Vec;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use tcs3472::Tcs3472;
extern crate alloc;

const CONFIG_BUFFER_SIZE: usize = 4096;
const DEFAULT_LEDS_AMOUNT: usize = default_from_env!(DEFAULT_LEDS_AMOUNT, 16);
// Magic inside - see andon_light_core docs to understand why '* 12' is here
const MAX_SUPPORTED_LEDS: usize = default_from_env!(MAX_SUPPORTED_LEDS, 16) * 12;

#[inline]
const fn _default_leds() -> usize {
    DEFAULT_LEDS_AMOUNT
}

#[inline]
const fn _default_brightness() -> u8 {
    10
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

#[derive(Debug, Hash, Eq, PartialEq, Clone, ErrorCodesEnum)]
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
    #[code(message = "No data from device", level = "error")]
    E002,
    #[code(message = "Device data too ambigous", level = "error")]
    E003,
}

type AndonLight =
    andon_light_core::AndonLight<ErrorCodes, { ErrorCodes::MIN_SET_SIZE }, MAX_SUPPORTED_LEDS>;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VersionProbe {
    version: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DeviceConfig {
    // Maybe use string for version in future?
    #[serde(default = "_default_version")]
    version: u32,
    #[serde(default = "_default_true")]
    buzzer_enabled: bool,
    #[serde(default = "_default_leds")]
    leds_amount: usize,
    #[serde(default = "_default_brightness")]
    brightness: u8,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        serde_json_core::de::from_slice(b"{}").unwrap().0
    }
}

impl DeviceConfig {
    pub fn generate_andon_config(&self) -> andon_light_core::Config {
        andon_light_core::Config::new(self.leds_amount as u8, self.brightness)
    }
}

struct Device {
    config: DeviceConfig,
    andon_light: AndonLight,
}

impl Device {
    pub fn new(device_config: DeviceConfig) -> Self {
        let andon_light = AndonLight::new(device_config.generate_andon_config());
        Self {
            config: device_config,
            andon_light,
        }
    }
}

type DeviceAsyncMutex = Mutex<CriticalSectionRawMutex, Device>;

#[embassy_executor::task]
async fn run_andon_light(
    mut spi: SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>,
    device: &'static DeviceAsyncMutex,
    mut led_reset: Output<'static>,
) {
    log::debug!("Resetting leds");
    led_reset.set_high();
    led_reset.set_low();

    log::debug!("Start of led test procedure");

    let test_patterns;
    let pause: u64 = 150;

    {
        let mut device = device.lock().await;
        test_patterns = device.andon_light.generate_test_patterns();
    }
    for pattern in test_patterns {
        spi.write(pattern.as_slice()).await.unwrap();
        Timer::after(Duration::from_millis(pause)).await;
    }
    Timer::after(Duration::from_millis(pause)).await;
    log::debug!("End of test procedure");

    log::debug!("Entering regular leds cycle");
    loop {
        let pattern;
        {
            let mut device = device.lock().await;
            pattern = device.andon_light.generate_signal();
        }
        spi.write(pattern.as_slice()).await.unwrap();
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

// TODO: Make buzzer task more efficient
#[embassy_executor::task]
async fn buzzer(
    mut buzzer_pin: GpioPin<3>,
    ledc: Ledc<'static>,
    device: &'static DeviceAsyncMutex,
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
    type Melody = Vec<(&'static str, usize), 5>;
    let staurtup_tune: Melody =
        Vec::from_slice(&[("d4", 1), ("f4", 1), ("a4", 1), ("d5", 1), (" ", 2)]).unwrap();
    let chill_tune: Melody = Vec::from_slice(&[(" ", 1)]).unwrap();
    let attentive_tune: Melody = Vec::from_slice(&[("c4", 1), (" ", 100)]).unwrap();
    let important_tune: Melody = Vec::from_slice(&[("c5", 1), (" ", 10)]).unwrap();
    let urgent_tune: Melody = Vec::from_slice(&[("c6", 1), (" ", 1)]).unwrap();

    let mut tune = &staurtup_tune;
    let mut first_run = true;
    let mut last_level: Option<AlertLevel> = None;
    loop {
        'melody: for note in tune {
            if !first_run {
                let device = device.lock().await;
                let alert_level = device.andon_light.calculate_alert_level();
                let recalc;
                if let Some(level) = &last_level {
                    recalc = *level != alert_level;
                } else {
                    recalc = true;
                }
                if recalc {
                    match alert_level {
                        AlertLevel::Chill => {
                            tune = &chill_tune;
                        }
                        AlertLevel::Attentive => {
                            tune = &attentive_tune;
                        }
                        AlertLevel::Important => {
                            tune = &important_tune;
                        }
                        AlertLevel::Urgent => {
                            tune = &urgent_tune;
                        }
                    }
                    last_level = Some(alert_level);
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
                        _ => 5,
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
                        if !first_run {
                            // TODO: Checking for alert level in the same "thread" as buzzer task
                            // may block (or disort) playing melody - need to find a way to divide
                            // it.
                            let device = device.lock().await;
                            let alert_level = device.andon_light.calculate_alert_level();
                            match &last_level {
                                Some(level) if *level != alert_level => break 'melody,
                                _ => {}
                            };
                        }
                    }
                }
            }
        }
        if first_run {
            first_run = false;
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

#[embassy_executor::task]
async fn rgb_probe_task(
    mut sensor: Tcs3472<I2c<'static, Async>>,
    device: &'static DeviceAsyncMutex,
) {
    log::debug!("RGB probe task started");
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mapper = ColorMapper::default();
    loop {
        'main: loop {
            match (
                sensor.enable().await,
                device.lock().await,
                ErrorCodes::SE001,
            ) {
                (Err(_), mut device, code) => {
                    device.andon_light.notify(code);
                    break 'main;
                }
                (Ok(_), mut device, code) => {
                    device.andon_light.resolve(code);
                }
            }

            match (
                sensor.enable_rgbc().await,
                device.lock().await,
                ErrorCodes::SE002,
            ) {
                (Err(_), mut device, code) => {
                    device.andon_light.notify(code);
                    break 'main;
                }
                (Ok(_), mut device, code) => {
                    device.andon_light.resolve(code);
                }
            }

            while !sensor.is_rgbc_status_valid().await.unwrap() {
                // wait for measurement to be available
                Timer::after(Duration::from_millis(10)).await;
            }

            let exclusive = Vec::<ErrorCodes, 6>::from_slice(&[
                ErrorCodes::SE003,
                ErrorCodes::E001,
                ErrorCodes::E002,
                ErrorCodes::E003,
                ErrorCodes::I001,
                ErrorCodes::W001,
            ])
            .unwrap();

            'inner: loop {
                match sensor.read_all_channels().await {
                    Err(_) => {
                        let mut device = device.lock().await;
                        device.andon_light.notify(ErrorCodes::SE003);
                        break 'inner;
                    }
                    Ok(measurement) => {
                        let color = mapper.translate_proportionally(
                            measurement.red,
                            measurement.green,
                            measurement.blue,
                        );
                        {
                            log::trace!("Color detected: {:?}", color);
                            let mut device = device.lock().await;
                            device.andon_light.resolve(ErrorCodes::SE003);
                            // This color schema is made for Prusa MK4 - in future I hope there
                            // will be more schemas available, but for now this is hardcoded
                            let error_code = match color {
                                Color::Green | Color::Mint | Color::Lime | Color::Black => {
                                    ErrorCodes::I001
                                } // Idle
                                Color::Blue | Color::Violet | Color::Azure | Color::Cyan => {
                                    ErrorCodes::Ok // Working
                                }
                                Color::Yellow | Color::Orange => ErrorCodes::W001, // Warning
                                Color::Red | Color::Pink | Color::Magenta => ErrorCodes::E001, // Critical error
                                Color::Gray | Color::White => ErrorCodes::E003, // Ambigous data
                            };
                            device.andon_light.notify_exclusive(error_code, &exclusive);
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

    // Initialization of wifi - for now it is only stub to test that its working, so with with next
    // software update it will be able to actually connect to wifi and do something useful
    let wifi_ctrl = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let (mut controller, _interfaces) = esp_wifi::wifi::new(&wifi_ctrl, peripherals.WIFI).unwrap();

    controller.set_mode(wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();
    let aps: (Vec<wifi::AccessPointInfo, 20>, usize) = controller.scan_n().unwrap();
    for ap in aps.0 {
        log::info!("Found AP: {:?}", ap);
    }

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

    #[allow(clippy::manual_div_ceil)]
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
    let result = sd_reader.read_config("CONFIG.JSO", &mut buffer).await;

    let app_config = match result {
        Ok(num_read) => {
            // For now only checking version, but in future it may be used to check compatibility
            match serde_json_core::de::from_slice::<VersionProbe>(&buffer[0..num_read]) {
                Ok((result, _)) => {
                    log::info!("Version read properly from file: {}", result.version);
                }
                _ => {
                    log::info!("Seems there is no version in config file, falling back to default");
                }
            }

            match serde_json_core::de::from_slice::<DeviceConfig>(&buffer[0..num_read]) {
                Ok((result, _)) => {
                    log::debug!("Config read properly from file");
                    result
                }
                Err(e) => {
                    log::warn!("Seems there is no config file, falling back to default");
                    log::warn!("{:?}", e);
                    DeviceConfig::default()
                }
            }
        }
        Err(e) => {
            log::warn!("Failed to read config file");
            log::warn!("{:?}", e);
            DeviceConfig::default()
        }
    };

    log::debug!("End of config read");
    log::debug!("Starting up leds control");

    let leds_select = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let spi_dev = SpiDevice::new(spi_bus, leds_select);

    let led_reset = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    static ANDON: StaticCell<DeviceAsyncMutex> = StaticCell::new();

    let device = ANDON.init(Mutex::new(Device::new(app_config)));
    spawner
        .spawn(run_andon_light(spi_dev, device, led_reset))
        .unwrap();

    log::debug!("Leds started");

    let buzzer_enabled;
    {
        buzzer_enabled = device.lock().await.config.buzzer_enabled;
    }
    // Buzzer
    if buzzer_enabled {
        let buzzer_pin = peripherals.GPIO3;
        spawner.spawn(buzzer(buzzer_pin, ledc, device)).unwrap();
    } else {
        log::debug!("Buzzer disabled");
    }

    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let sensor = Tcs3472::new(i2c);
    spawner.spawn(rgb_probe_task(sensor, device)).unwrap();
}

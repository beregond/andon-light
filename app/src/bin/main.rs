#![no_std]
#![no_main]

use andon_light::*;

use andon_light_core::{
    colors::{Color, ColorMapper},
    AlertLevel, DeviceState, ErrorCodesBase, SystemState,
};
use andon_light_macros::{default_from_env, ErrorCodesEnum};
use core::fmt::Write;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{dns::DnsQueryType, tcp::TcpSocket};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::spi::SpiDevice as _;
use esp_backtrace as _;
use esp_hal::efuse::Efuse;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::peripherals::GPIO3;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::I2c,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    rng::Rng,
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use esp_radio::wifi::{
    ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState,
};
use heapless::Vec;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use tcs3472::Tcs3472;
extern crate alloc;
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Receiver, Sender, Watch};
use rust_mqtt::{
    client::client::MqttClient,
    client::client_config::{ClientConfig as MqttClientConfig, MqttVersion::MQTTv5},
    packet::v5::{publish_packet::QualityOfService::QoS1, reason_codes::ReasonCode},
    utils::rng_generator::CountingRng,
};

esp_bootloader_esp_idf::esp_app_desc!();

const VERSION: &str = env!("CARGO_PKG_VERSION");
const BUILD_VERSION: &str = env!("VERGEN_GIT_DESCRIBE");
const BUILD_TIME: &str = env!("VERGEN_BUILD_TIMESTAMP");

#[derive(Debug)]
pub enum WifiState {
    Disconnected,
    Connected,
}

const CONFIG_BUFFER_SIZE: usize = 4096;
const LEDS_AMOUNT: usize = default_from_env!(LEDS_AMOUNT, 16);
// Magic inside - see andon_light_core docs to understand why '* 12' is here
const MAX_SUPPORTED_LEDS: usize = default_from_env!(MAX_SUPPORTED_LEDS, 16) * 12;

#[inline]
const fn _empty_str() -> &'static str {
    ""
}

#[inline]
const fn _cnc() -> &'static str {
    "cnc"
}

#[inline]
const fn _andon() -> &'static str {
    "andon"
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

#[inline]
const fn _default_mqtt_port() -> u16 {
    1883
}

#[inline]
fn andon_id() -> heapless::String<32> {
    let mac = Efuse::read_base_mac_address();
    let mut id = heapless::String::<32>::new();
    write!(id, "light-").unwrap();
    for chunk in mac {
        write!(id, "{:02X}", chunk).unwrap();
    }
    id
}

type SpiBus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;

#[derive(Debug, Hash, Eq, PartialEq, Clone, ErrorCodesEnum, Copy, Serialize)]
pub enum ErrorCodes {
    #[code(message = "Ok", level = "ok")]
    Ok,
    #[code(message = "Device is idle", level = "idle")]
    I001,
    #[code(message = "Device is turned off", level = "idle")]
    I002,
    #[code(message = "Device reports warning", level = "warn")]
    W001,
    #[code(message = "TCS sensor is unavailable", level = "system_error")]
    SE001,
    #[code(message = "WiFi disconnected", level = "system_warn")]
    SW001,
    #[code(message = "WiFi could not connect", level = "system_warn")]
    SW002,
    #[code(message = "MQTT config error", level = "system_warn")]
    SW003,
    #[code(message = "MQTT encountered DNS error", level = "system_warn")]
    SW004,
    #[code(message = "MQTT network connection error", level = "system_warn")]
    SW005,
    #[code(message = "MQTT connection to broker failed", level = "system_warn")]
    SW006,
    #[code(message = "MQTT couldn't send message", level = "system_warn")]
    SW007,
    #[code(message = "MQTT couldn't send ping", level = "system_warn")]
    SW008,
    #[code(message = "Failed to enable TCS sensor", level = "system_error")]
    SE002,
    #[code(message = "TCS sensor error", level = "system_error")]
    SE003,
    #[code(message = "Device reported error", level = "error")]
    E001,
    #[code(message = "No data from device", level = "error")]
    E002,
    #[code(message = "Device data too ambigous", level = "error")]
    E003,
}

type AndonLight = andon_light_core::AndonLight<
    ErrorCodes,
    { ErrorCodes::CODES_AMOUNT },
    { ErrorCodes::MIN_SET_SIZE },
    MAX_SUPPORTED_LEDS,
>;

// NOTE: this must be declared here for now, as IAT are in progress,
// check https://github.com/rust-lang/rust/issues/8995
type AndonReport = (
    SystemState,
    DeviceState,
    AlertLevel,
    heapless::Vec<ErrorCodes, { ErrorCodes::CODES_AMOUNT }>,
);

// One for buzzer, one for mqtt
const ANDON_REPORT_CHANNELS: usize = 2;
type AndonReporter = Watch<CriticalSectionRawMutex, AndonReport, ANDON_REPORT_CHANNELS>;
type AndonReporterSender =
    Sender<'static, CriticalSectionRawMutex, AndonReport, ANDON_REPORT_CHANNELS>;
type AndonReporterReceiver =
    Receiver<'static, CriticalSectionRawMutex, AndonReport, ANDON_REPORT_CHANNELS>;

// Onef for anon light runner, one for buzzer, one for mqtt
const ANDON_NOTIFIER_CHANNELS: usize = 3;
type AndonNotifier = Watch<CriticalSectionRawMutex, bool, ANDON_NOTIFIER_CHANNELS>;
type AndonNotifierSender = Sender<'static, CriticalSectionRawMutex, bool, ANDON_NOTIFIER_CHANNELS>;
type AndonNotifierReceiver =
    Receiver<'static, CriticalSectionRawMutex, bool, ANDON_NOTIFIER_CHANNELS>;

type WifiSignal = Signal<CriticalSectionRawMutex, WifiState>;
static ANDON_NOTIFIER: AndonNotifier = Watch::new();
static ANDON_REPORTER: AndonReporter = Watch::new();
static WIFI_SIGNAL: WifiSignal = Signal::new();

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VersionProbe {
    version: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct DeviceConfig {
    // Maybe use string for version in future?
    #[serde(default = "_default_version")]
    version: u32,
    #[serde(default = "_empty_str")]
    id: &'static str,
    #[serde(default = "_default_true")]
    buzzer_enabled: bool,
    #[serde(default = "_default_brightness")]
    brightness: u8,
    #[serde(default = "_empty_str")]
    wifi_ssid: &'static str,
    #[serde(default = "_empty_str")]
    wifi_password: &'static str,
    #[serde(default = "_empty_str")]
    mqtt_host: &'static str,
    #[serde(default = "_default_mqtt_port")]
    mqtt_port: u16,
    #[serde(default = "_empty_str")]
    mqtt_username: &'static str,
    #[serde(default = "_empty_str")]
    mqtt_password: &'static str,
    #[serde(default = "_cnc")]
    mqtt_device_type: &'static str,
    #[serde(default = "_andon")]
    mqtt_topic_prefix: &'static str,
    #[serde(default = "_empty_str")]
    mqtt_topic: &'static str,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        serde_json_core::de::from_slice(b"{}").unwrap().0
    }
}

impl DeviceConfig {
    pub fn generate_andon_config(&self) -> andon_light_core::Config {
        andon_light_core::Config::new(LEDS_AMOUNT as u8, self.brightness)
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

macro_rules! notify {
    ($device:expr, $error_code:expr) => {{
        let mut device = $device.lock().await;
        device.andon_light.notify($error_code);
    }};
}
macro_rules! resolve {
    ($device:expr, $error_code:expr) => {{
        let mut device = $device.lock().await;
        device.andon_light.resolve($error_code);
    }};
}

#[embassy_executor::task]
async fn run_andon_light(
    mut spi: SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>,
    device: &'static DeviceAsyncMutex,
    mut andon_notifier: AndonNotifierReceiver,
) {
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
        select(
            Timer::after(Duration::from_secs(10)),
            andon_notifier.changed(),
        )
        .await;
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
            drive_mode: DriveMode::PushPull,
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

/// Checks for alert level changes and signals to other peripherals,
/// so they can react accordingly
#[embassy_executor::task]
async fn andon_supervisor(
    device: &'static DeviceAsyncMutex,
    notifier: AndonNotifierSender,
    reporter: AndonReporterSender,
) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut current_report;
    {
        let device = device.lock().await;
        current_report = device.andon_light.get_report()
    }
    loop {
        let report;
        {
            let device = device.lock().await;
            report = device.andon_light.get_report();
        }
        if current_report != report {
            reporter.send(report.clone());
            notifier.send(true);
            current_report = report;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn buzzer(
    mut buzzer_pin: GPIO3<'static>,
    ledc: Ledc<'static>,
    device: &'static DeviceAsyncMutex,
    mut andon_notifier: AndonNotifierReceiver,
    mut andon_reporter: AndonReporterReceiver,
) {
    let tones = [
        ("c4", 262),
        ("d4", 294),
        ("f4", 349),
        ("a4", 440),
        ("c5", 523),
        ("d5", 587),
        ("c6", 1046),
        // Although we set "silent" note here - frequency must not be less than 256, or Divisor
        // error will be returned from lstimer.
        (" ", 256),
    ];
    type Melody = Vec<(&'static str, usize), 5>;
    let staurtup_tune: Melody =
        Vec::from_slice(&[("d4", 1), ("f4", 1), ("a4", 1), ("d5", 1), (" ", 2)]).unwrap();
    let chill_tune: Melody = Vec::from_slice(&[(" ", 1)]).unwrap();
    let attentive_tune: Melody = Vec::from_slice(&[("c4", 1), (" ", 100)]).unwrap();
    let important_tune: Melody = Vec::from_slice(&[("c5", 1), (" ", 10)]).unwrap();
    let urgent_tune: Melody = Vec::from_slice(&[("c6", 1), (" ", 1)]).unwrap();

    let mut tune = &staurtup_tune;
    let mut last_level: Option<AlertLevel> = None;
    let mut switch_to_alert_level: Option<AlertLevel> = None;
    loop {
        'melody: for note in tune {
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
                    let mut channel =
                        ledc.channel(channel::Number::Channel1, buzzer_pin.reborrow());
                    channel
                        .configure(channel::config::Config {
                            timer: &lstimer,
                            duty_pct,
                            drive_mode: DriveMode::PushPull,
                        })
                        .unwrap();
                    for _ in 0..note.1 {
                        Timer::after(Duration::from_millis(100)).await;
                        // 'is_some' points to the fact that we are NOT in startup tune
                        // (only after test tune this variable will be set)
                        // There is funny egde case when this task and supervisor starts at the
                        // same time and reads urgent level at the "same" time, because
                        // you will end up in playing longer beep after startup tune.
                        // To avoid it - only check for new signal after at leas one 'silent'
                        // note,so the sound will not compile in longer beep (thats wy we check
                        // note.0 == " ")
                        // Additionally we check if level is different, again because of race
                        // condition
                        if note.0 == " " {
                            if let Some(level) = last_level {
                                if andon_notifier.try_changed().is_some() {
                                    if let Some(report) = andon_reporter.try_changed() {
                                        if report.2 != level {
                                            switch_to_alert_level = Some(report.2);
                                            break 'melody;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        let alert_level = match (switch_to_alert_level, andon_reporter.try_changed()) {
            (Some(level), Some(_)) | (Some(level), None) => level,
            (None, Some(report)) => {
                // Preemptive drain
                andon_notifier.try_changed();
                report.2
            }
            (None, None) => {
                let device = device.lock().await;
                device.andon_light.calculate_alert_level()
            }
        };
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
    }
}

#[embassy_executor::task]
async fn rgb_probe_task(
    mut sensor: Tcs3472<I2c<'static, Async>>,
    device: &'static DeviceAsyncMutex,
) {
    log::debug!("RGB probe task started");
    let mut ticker = Ticker::every(Duration::from_millis(500));
    // TODO: allow to configure color mapper params from file?
    let mapper = ColorMapper::default();
    let exclusive = Vec::<ErrorCodes, 7>::from_slice(&[
        ErrorCodes::SE003,
        ErrorCodes::E001,
        ErrorCodes::E002,
        ErrorCodes::E003,
        ErrorCodes::I001,
        ErrorCodes::I002,
        ErrorCodes::W001,
    ])
    .unwrap();
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
                            log::trace!("Color detected: {color:?}");
                            // This color schema is made for Prusa MK4 - in future I hope there
                            // will be more schemas available, but for now this is hardcoded
                            let error_code = match color {
                                Color::Black => ErrorCodes::I002, // No data, device is turned off
                                Color::Green | Color::Mint | Color::Lime => ErrorCodes::I001, // Idle
                                Color::Blue | Color::Violet | Color::Azure | Color::Cyan => {
                                    ErrorCodes::Ok // Working
                                }
                                Color::Yellow | Color::Orange => ErrorCodes::W001, // Warning
                                Color::Red | Color::Pink | Color::Magenta => ErrorCodes::E001, // Critical error
                                Color::Gray | Color::White => ErrorCodes::E003, // Ambigous data
                            };
                            let mut device = device.lock().await;
                            device.andon_light.resolve(ErrorCodes::SE003);
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

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    log::info!("Hello world!");
    log::info!("Andon Light firmware version: {}", BUILD_VERSION);
    log::info!("Build timestamp: {}", BUILD_TIME);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    log::info!("Embassy initialized!");

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

    static BUFFER: StaticCell<[u8; CONFIG_BUFFER_SIZE]> = StaticCell::new();
    let buffer = BUFFER.init([0u8; CONFIG_BUFFER_SIZE]);
    let sd_select = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    let sd_reader = SdReader::new(spi_bus, sd_select, Delay::new());
    // TODO: Need better support for filesystem and config errors
    let result = sd_reader.read_config("CONFIG.JSO", buffer).await;

    let mut app_config = match result {
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
                    log::warn!("{e:?}");
                    DeviceConfig::default()
                }
            }
        }
        Err(e) => {
            log::warn!("Failed to read config file");
            log::warn!("{e:?}");
            DeviceConfig::default()
        }
    };

    if app_config.id.is_empty() {
        static ID_BUFFER: StaticCell<heapless::String<32>> = StaticCell::new();
        app_config.id = ID_BUFFER.init(andon_id());
    }

    log::debug!("End of config read");
    log::info!("Andon light ID: {}", app_config.id);

    log::debug!("Starting up leds control");

    let leds_select = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());
    let spi_dev = SpiDevice::new(spi_bus, leds_select);

    static ANDON: StaticCell<DeviceAsyncMutex> = StaticCell::new();

    let notifier = ANDON_NOTIFIER.sender();
    let reporter = ANDON_REPORTER.sender();
    let device = ANDON.init(Mutex::new(Device::new(app_config)));
    spawner
        .spawn(run_andon_light(
            spi_dev,
            device,
            ANDON_NOTIFIER.receiver().unwrap(),
        ))
        .unwrap();

    log::debug!("Leds started");

    spawner
        .spawn(andon_supervisor(device, notifier, reporter))
        .unwrap();

    {
        let andon = device.lock().await;
        if andon.config.buzzer_enabled {
            log::debug!("Buzzer enabled");
            let buzzer_pin = peripherals.GPIO3;
            spawner
                .spawn(buzzer(
                    buzzer_pin,
                    ledc,
                    device,
                    ANDON_NOTIFIER.receiver().unwrap(),
                    ANDON_REPORTER.receiver().unwrap(),
                ))
                .unwrap();
        } else {
            log::debug!("Buzzer is disabled");
        }
    }

    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let sensor = Tcs3472::new(i2c);
    spawner.spawn(rgb_probe_task(sensor, device)).unwrap();

    // Initialization of wifi
    {
        let andon = device.lock().await;
        if !andon.config.wifi_ssid.is_empty() && !andon.config.wifi_password.is_empty() {
            static NET_RESOURCES: StaticCell<esp_radio::Controller> = StaticCell::new();
            log::info!("Connecting to WiFi SSID: {}", andon.config.wifi_ssid);
            let esp_radio_ctrl = NET_RESOURCES.init(esp_radio::init().unwrap());

            // We must initialize some kind of interface and start it.
            let (controller, interfaces) =
                esp_radio::wifi::new(esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();

            let wifi_interface = interfaces.sta;
            let config = embassy_net::Config::dhcpv4(Default::default());
            let rng = Rng::new();
            let seed = (rng.random() as u64) << 32 | rng.random() as u64;

            // Init network stack
            static RESOURCES: StaticCell<embassy_net::StackResources<5>> = StaticCell::new();
            let (stack, runner) = embassy_net::new(
                wifi_interface,
                config,
                RESOURCES.init(embassy_net::StackResources::new()),
                seed,
            );

            let mode_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(andon.config.wifi_ssid.into())
                    .with_password(andon.config.wifi_password.into()),
            );

            spawner.spawn(net_task(runner)).ok();
            spawner
                .spawn(connection(
                    controller,
                    device,
                    stack,
                    mode_config,
                    &WIFI_SIGNAL,
                ))
                .ok();

            if !andon.config.mqtt_host.is_empty() {
                spawner
                    .spawn(mqtt_publisher(
                        device,
                        stack,
                        &WIFI_SIGNAL,
                        ANDON_NOTIFIER.receiver().unwrap(),
                        ANDON_REPORTER.receiver().unwrap(),
                    ))
                    .ok();
            } else {
                log::info!("MQTT host is empty, skipping MQTT initialization");
            }
        } else {
            log::info!("WiFi SSID or password is empty, skipping WiFi initialization, skipping MQTT initialization");
        }
    }
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    device: &'static DeviceAsyncMutex,
    stack: embassy_net::Stack<'static>,
    client_config: ModeConfig,
    wifi_signal: &'static WifiSignal,
) {
    let exclusive =
        Vec::<ErrorCodes, 2>::from_slice(&[ErrorCodes::SW001, ErrorCodes::SW002]).unwrap();
    log::debug!("start connection task");
    log::debug!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if let WifiStaState::Connected = esp_radio::wifi::sta_state() {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            notify!(device, ErrorCodes::SW002);
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            controller.set_config(&client_config).unwrap();
            log::debug!("Starting wifi");
            controller.start_async().await.unwrap();
            log::debug!("Wifi started!");
        }
        // TODO: await below is ONLY to postpone `connect_async` which takes long and makes
        // test procedure seem to lag
        Timer::after(Duration::from_millis(2000)).await;
        log::debug!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => {
                log::debug!("Wifi connected!");
                loop {
                    if stack.is_link_up() {
                        break;
                    }
                    Timer::after(Duration::from_millis(500)).await;
                }

                log::debug!("Waiting to get IP address...");
                loop {
                    if let Some(config) = stack.config_v4() {
                        log::debug!("Got IP: {}", config.address);
                        break;
                    }
                    Timer::after(Duration::from_millis(500)).await;
                }
                {
                    let mut device = device.lock().await;
                    device.andon_light.resolve_all(&exclusive);
                }

                wifi_signal.signal(WifiState::Connected);
            }
            Err(e) => {
                log::debug!("Failed to connect to wifi: {e:?}");
                notify!(device, ErrorCodes::SW001);
                wifi_signal.signal(WifiState::Disconnected);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[derive(Serialize)]
struct MqttMessage<'a> {
    id: &'a str,
    device_state: &'a str,
    system_state: &'a str,
    codes: heapless::Vec<ErrorCodes, { ErrorCodes::CODES_AMOUNT }>,
}

#[derive(Serialize)]
struct MqttStatusMessage<'a> {
    id: &'a str,
    status: &'a str,
    version: Option<&'a str>,
}

#[embassy_executor::task]
async fn mqtt_publisher(
    device: &'static DeviceAsyncMutex,
    stack: embassy_net::Stack<'static>,
    wifi_signal: &'static WifiSignal,
    mut andon_notifier: AndonNotifierReceiver,
    mut andon_reporter: AndonReporterReceiver,
) {
    let mut root_topic = heapless::String::<100>::new();
    let andon_id;
    {
        let andon = device.lock().await;
        let cc = &andon.config;
        andon_id = cc.id;
        match (
            cc.mqtt_topic,
            cc.mqtt_topic_prefix,
            cc.mqtt_device_type,
            cc.id,
        ) {
            (topic, _, _, _) if !topic.is_empty() => {
                write!(root_topic, "{}", topic).unwrap();
            }
            // Validation,
            ("", _, "", _) | ("", _, _, "") => {
                log::error!("MQTT topic parts are not properly configured, cannot proceed with MQTT publisher");
                notify!(device, ErrorCodes::SW003);
                return;
            }
            // Prefix may be unset
            ("", "", device_type, andon_id) => {
                write!(root_topic, "{}/{}", device_type, andon_id).unwrap();
            }
            ("", prefix, device_type, andon_id) => {
                write!(root_topic, "{}/{}/{}", prefix, device_type, andon_id).unwrap();
            }
            _ => {
                log::error!("Unexpected MQTT configuration");
                notify!(device, ErrorCodes::SW003);
                return;
            }
        }
    }
    // NOTE: No resolve here for SW003, as we are having hard return above

    let mut current_state: WifiState;
    let lwt_payload: serde_json_core::heapless::String<70> =
        serde_json_core::to_string(&MqttStatusMessage {
            id: andon_id,
            status: "offline",
            version: None,
        })
        .unwrap();
    let connection_topic = generate_mqtt_topic(&root_topic, "connection");
    let state_topic = generate_mqtt_topic(&root_topic, "state");

    'main: loop {
        loop {
            current_state = wifi_signal.wait().await;
            match current_state {
                WifiState::Connected => {
                    log::debug!("MQTT publisher detected WiFi connected, starting MQTT client...");
                    break;
                }
                // No warns here - they are handled in connection task
                WifiState::Disconnected => {
                    log::debug!("MQTT publisher detected WiFi disconnected, waiting...");
                }
            }
        }

        'connection: loop {
            if matches!(wifi_signal.try_take(), Some(WifiState::Disconnected)) {
                log::debug!(
                    "MQTT publisher detected WiFi disconnected, dropping MQTT connection if any..."
                );
                // No warns here - they are handled in connection task
                continue 'main;
            }
            let mut rx_buffer = [0; 4096];
            let mut tx_buffer = [0; 4096];

            let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(embassy_time::Duration::from_secs(30)));
            let mut config;
            {
                let mut device = device.lock().await;
                let address = match stack
                    .dns_query(device.config.mqtt_host, DnsQueryType::A)
                    .await
                    .map(|a| a[0])
                {
                    Ok(address) => address,
                    Err(e) => {
                        log::error!("DNS lookup error: {e:?}");
                        device.andon_light.notify(ErrorCodes::SW004);
                        continue 'connection;
                    }
                };
                device.andon_light.resolve(ErrorCodes::SW004);
                let remote_endpoint = (address, device.config.mqtt_port);
                log::debug!("MQTT connecting to {remote_endpoint:?}...");
                if let Err(e) = socket.connect(remote_endpoint).await {
                    log::error!("MQTT connect error: {:?}", e);
                    device.andon_light.notify(ErrorCodes::SW005);
                    continue 'connection;
                }
                device.andon_light.resolve(ErrorCodes::SW005);
                log::debug!("MQTT TCP connection established");

                config = MqttClientConfig::<5, CountingRng>::new(MQTTv5, CountingRng(20000));
                // TODO: Move some of these to consts
                config.add_max_subscribe_qos(QoS1);
                config.add_client_id(andon_id);
                config.max_packet_size = 100;
                config.keep_alive = 60;
                config.add_will(&connection_topic, lwt_payload.as_bytes(), true);

                if !device.config.mqtt_username.is_empty()
                    && !device.config.mqtt_password.is_empty()
                {
                    log::debug!("Using MQTT username: {}", device.config.mqtt_username);
                    config.add_username(device.config.mqtt_username);
                    config.add_password(device.config.mqtt_password);
                } else {
                    log::debug!("No MQTT username/password provided, connecting anonymously");
                }
            }

            let mut writebuf = [0; 1024];
            let mut readbuf = [0; 1024];
            // TODO: Bound properties to consts
            let mut client =
                MqttClient::<_, 5, _>::new(socket, &mut writebuf, 200, &mut readbuf, 200, config);

            match client.connect_to_broker().await {
                Ok(()) => {
                    log::info!("Connected to MQTT broker, starting publishing messages");
                }
                Err(mqtt_error) => {
                    if let ReasonCode::NetworkError = mqtt_error {
                        log::error!("MQTT Network Error");
                    } else {
                        log::error!("Other MQTT Error: {:?}", mqtt_error);
                    }
                    notify!(device, ErrorCodes::SW006);
                    continue 'connection;
                }
            }
            resolve!(device, ErrorCodes::SW006);

            let message: serde_json_core::heapless::String<100> =
                serde_json_core::to_string(&MqttStatusMessage {
                    id: andon_id,
                    status: "online",
                    version: Some(VERSION),
                })
                // TODO: handle error
                .unwrap();

            if send_mqtt_message(&mut client, &connection_topic, message.as_bytes())
                .await
                .is_err()
            {
                notify!(device, ErrorCodes::SW007);
                continue 'connection;
            } else {
                resolve!(device, ErrorCodes::SW007);
            }

            let andon_state;
            let error_codes;
            match andon_reporter.try_changed() {
                Some(report) => {
                    andon_state = (report.0, report.1);
                    error_codes = report.3;
                }
                None => {
                    let device = device.lock().await;
                    andon_state = device.andon_light.get_state();
                    error_codes = device.andon_light.get_codes();
                }
            }

            let first_message = MqttMessage {
                id: andon_id,
                system_state: andon_state.0.as_str(),
                device_state: andon_state.1.as_str(),
                codes: error_codes,
            };
            // TODO: Can I get rid of size notation?
            let payload: serde_json_core::heapless::String<100> =
                serde_json_core::to_string(&first_message).unwrap();

            if send_mqtt_message(&mut client, &state_topic, payload.as_bytes())
                .await
                .is_err()
            {
                notify!(device, ErrorCodes::SW007);
                continue 'connection;
            } else {
                resolve!(device, ErrorCodes::SW007);
            }

            loop {
                let timer = Timer::after(Duration::from_secs(20));
                select(timer, andon_notifier.changed()).await;
                match andon_reporter.try_changed() {
                    Some(report) => {
                        let message = MqttMessage {
                            id: andon_id,
                            system_state: report.0.as_str(),
                            device_state: report.1.as_str(),
                            codes: report.3,
                        };
                        let another_payload: serde_json_core::heapless::String<100> =
                            serde_json_core::to_string(&message).unwrap();

                        if send_mqtt_message(&mut client, &state_topic, another_payload.as_bytes())
                            .await
                            .is_err()
                        {
                            notify!(device, ErrorCodes::SW007);
                            continue 'connection;
                        }
                    }
                    None => {
                        if let Err(error) = client.send_ping().await {
                            log::error!("Failed to send MQTT ping: {:?}", error);
                            notify!(device, ErrorCodes::SW007);
                            continue 'connection;
                        }
                    }
                }
                {
                    let mut device = device.lock().await;
                    device.andon_light.resolve(ErrorCodes::SW007);
                    device.andon_light.resolve(ErrorCodes::SW008);
                }
            }
        }
    }
}

pub fn generate_mqtt_topic(root_topic: &str, suffix: &str) -> heapless::String<60> {
    let mut topic = heapless::String::<60>::new();
    write!(topic, "{}/{}", root_topic, suffix).unwrap();
    topic
}

pub async fn send_mqtt_message(
    client: &mut MqttClient<'_, TcpSocket<'_>, 5, CountingRng>,
    topic: &str,
    payload: &[u8],
) -> Result<(), ReasonCode> {
    match client.send_message(topic, payload, QoS1, true).await {
        Ok(()) => {
            log::trace!("MQTT message sent successfully");
            Ok(())
        }
        Err(mqtt_error) => match mqtt_error {
            ReasonCode::NoMatchingSubscribers => {
                log::debug!("MQTT message sent successfully, but nobody is listening");
                Ok(()) // If nobody is listening, we still consider it successful
            }
            _ => {
                log::error!("Failed to send MQTT connection message: {:?}", mqtt_error);
                Err(mqtt_error)
            }
        },
    }
}

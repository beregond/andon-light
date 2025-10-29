#![no_std]
#![no_main]

use andon_light::*;

use andon_light_core::{
    colors::{Color, ColorMapper},
    AlertLevel, ErrorCodesBase,
};
use andon_light_macros::{default_from_env, ErrorCodesEnum};
use core::fmt::Write;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
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
use rust_mqtt::{
    client::client::MqttClient,
    packet::v5::{publish_packet::QualityOfService::QoS1, reason_codes::ReasonCode},
    utils::rng_generator::CountingRng,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Debug)]
pub enum WifiState {
    Disconnected,
    Connected,
}

type BuzzerSignal = Signal<CriticalSectionRawMutex, AlertLevel>;
type WifiSignal = Signal<CriticalSectionRawMutex, WifiState>;
static BUZZER_SIGNAL: BuzzerSignal = Signal::new();
static WIFI_SIGNAL: WifiSignal = Signal::new();

const CONFIG_BUFFER_SIZE: usize = 4096;
const DEFAULT_LEDS_AMOUNT: usize = default_from_env!(DEFAULT_LEDS_AMOUNT, 16);
// Magic inside - see andon_light_core docs to understand why '* 12' is here
const MAX_SUPPORTED_LEDS: usize = default_from_env!(MAX_SUPPORTED_LEDS, 16) * 12;

#[inline]
const fn _default_leds() -> usize {
    DEFAULT_LEDS_AMOUNT
}

#[inline]
const fn _empty_str() -> &'static str {
    ""
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
fn device_id() -> heapless::String<12> {
    let mac = Efuse::read_base_mac_address();
    let mut s = heapless::String::<12>::new();
    // write!(s, "{:012X}", mac).unwrap();
    for b in mac {
        write!(s, "{:02X}", b).unwrap();
    }
    s
}

type SpiBus = Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>;

#[derive(Debug, Hash, Eq, PartialEq, Clone, ErrorCodesEnum, Copy)]
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
    #[serde(default = "_empty_str")]
    device_id: &'static str,
    #[serde(default = "_default_true")]
    buzzer_enabled: bool,
    #[serde(default = "_default_leds")]
    leds_amount: usize,
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

/// Checks for alert level changes and signals buzzer task if needed,
/// so buzzer task does not need to lock device mutex all the time, in between notes
#[embassy_executor::task]
async fn buzzer_supervisor(device: &'static DeviceAsyncMutex, sender: &'static BuzzerSignal) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut current_level;
    {
        let device = device.lock().await;
        if !device.config.buzzer_enabled {
            log::debug!("Buzzer is disabled, exiting supervisor task");
            return;
        }
        current_level = device.andon_light.calculate_alert_level();
    }
    loop {
        let alert_level;
        {
            let device = device.lock().await;
            alert_level = device.andon_light.calculate_alert_level();
        }
        if current_level != alert_level {
            current_level = alert_level;
            sender.signal(alert_level);
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn buzzer(
    mut buzzer_pin: GPIO3<'static>,
    ledc: Ledc<'static>,
    device: &'static DeviceAsyncMutex,
    signal: &'static BuzzerSignal,
) {
    {
        let device = device.lock().await;
        if device.config.buzzer_enabled {
            log::debug!("Buzzer enabled");
        } else {
            log::debug!("Buzzer is disabled, exiting task");
            return;
        }
    }
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
                        // There is funny egdg case when this task and supervisor starts at the
                        // same time and reads urgent level at the "same" time, because
                        // you will end up in playing longer beep after startup tune.
                        // To avoid it - only check for new signal after at leas one 'silent'
                        // note,so the sound will not compile in longer beep (thats wy we check
                        // note.0 == " ")
                        if last_level.is_some() && note.0 == " " {
                            let value = signal.try_take();
                            if value.is_some() {
                                // Since supervisor tracks changes, we can just switch to new level
                                // without checking what was the last one
                                switch_to_alert_level = value;
                                break 'melody;
                            }
                        }
                    }
                }
            }
        }
        let alert_level;
        if let Some(level) = switch_to_alert_level {
            alert_level = level;
            switch_to_alert_level = None;
        } else {
            // Since we have alert level from supervisor - this situation should not happen,
            // unless after startup tune - so it is justified to waif for device lock.
            let device = device.lock().await;
            alert_level = device.andon_light.calculate_alert_level();
        }
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
        // TODO: There is bug after update of esp-hal, when going into idle mode gives two notes in the beginning
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
                            log::trace!("Color detected: {color:?}");
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

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    log::info!("Hello world!");

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

    log::debug!("End of config read");

    log::debug!("Starting up leds control");

    let leds_select = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let spi_dev = SpiDevice::new(spi_bus, leds_select);

    let led_reset = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    static ANDON: StaticCell<DeviceAsyncMutex> = StaticCell::new();

    let device = ANDON.init(Mutex::new(Device::new(app_config.clone())));
    spawner
        .spawn(run_andon_light(spi_dev, device, led_reset))
        .unwrap();

    log::debug!("Leds started");

    // Buzzer
    spawner
        .spawn(buzzer_supervisor(device, &BUZZER_SIGNAL))
        .unwrap();
    let buzzer_pin = peripherals.GPIO3;
    spawner
        .spawn(buzzer(buzzer_pin, ledc, device, &BUZZER_SIGNAL))
        .unwrap();

    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let sensor = Tcs3472::new(i2c);
    spawner.spawn(rgb_probe_task(sensor, device)).unwrap();

    // Initialization of wifi
    if !app_config.wifi_ssid.is_empty() && !app_config.wifi_password.is_empty() {
        static NET_RESOURCES: StaticCell<esp_radio::Controller> = StaticCell::new();
        log::info!("Connecting to WiFi SSID: {}", app_config.wifi_ssid);
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
                .with_ssid(app_config.wifi_ssid.into())
                .with_password(app_config.wifi_password.into()),
        );

        spawner.spawn(net_task(runner)).ok();
        spawner
            .spawn(connection(controller, stack, mode_config, &WIFI_SIGNAL))
            .ok();

        if !app_config.mqtt_host.is_empty() {
            spawner
                .spawn(mqtt_publisher(
                    device,
                    stack,
                    &WIFI_SIGNAL,
                    app_config.mqtt_host,
                    app_config.mqtt_port,
                    app_config.mqtt_username,
                    app_config.mqtt_password,
                ))
                .ok();
        } else {
            log::info!("MQTT host is empty, skipping MQTT initialization");
        }
    } else {
        log::info!("WiFi SSID or password is empty, skipping WiFi initialization, skipping MQTT initialization");
    }
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    stack: embassy_net::Stack<'static>,
    client_config: ModeConfig,
    wifi_signal: &'static WifiSignal,
) {
    log::debug!("start connection task");
    log::debug!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if let WifiStaState::Connected = esp_radio::wifi::sta_state() {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
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
                wifi_signal.signal(WifiState::Connected);
            }
            Err(e) => {
                log::debug!("Failed to connect to wifi: {e:?}");
                wifi_signal.signal(WifiState::Disconnected);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn mqtt_publisher(
    device: &'static DeviceAsyncMutex,
    stack: embassy_net::Stack<'static>,
    wifi_signal: &'static WifiSignal,
    mqtt_host: &'static str,
    mqtt_port: u16,
    mqtt_username: &'static str,
    mqtt_password: &'static str,
) {
    let mut current_state: WifiState;
    let mut unique_id = heapless::String::<32>::new();
    write!(unique_id, "light-{}", device_id()).unwrap();
    let lwt_payload = generate_json_message(&unique_id, "offline");
    let system_topic = generate_mqtt_topic(&unique_id, "system");
    let device_topic = generate_mqtt_topic(&unique_id, "device");

    loop {
        current_state = wifi_signal.wait().await;
        match current_state {
            WifiState::Connected => {
                log::debug!("MQTT publisher detected WiFi connected, starting MQTT client...");
            }
            WifiState::Disconnected => {
                log::debug!("MQTT publisher detected WiFi disconnected, waiting...");
                continue;
            }
        }

        let mut rx_buffer = [0; 4096];
        let mut tx_buffer = [0; 4096];

        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(30)));
        // Put logic here
        let address = match stack
            .dns_query(mqtt_host, DnsQueryType::A)
            .await
            .map(|a| a[0])
        {
            Ok(address) => address,
            Err(e) => {
                log::debug!("DNS lookup error: {e:?}");
                // TODO: Better backoff strategy
                continue;
            }
        };
        let remote_endpoint = (address, mqtt_port);
        log::debug!("MQTT connecting to {remote_endpoint:?}...");
        if let Err(e) = socket.connect(remote_endpoint).await {
            log::debug!("connect error: {:?}", e);
            continue;
        }
        log::debug!("connected");

        // TODO: Typology
        let mut config: rust_mqtt::client::client_config::ClientConfig<'_, 5, CountingRng> =
            rust_mqtt::client::client_config::ClientConfig::new(
                rust_mqtt::client::client_config::MqttVersion::MQTTv5,
                CountingRng(20000),
            );
        config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
        config.add_client_id(&unique_id);
        config.max_packet_size = 100;
        config.keep_alive = 60;
        config.add_will(&system_topic, lwt_payload.as_bytes(), false);

        if !mqtt_username.is_empty() && !mqtt_password.is_empty() {
            log::debug!("Using MQTT username: {}", mqtt_username);
            config.add_username(mqtt_username);
            config.add_password(mqtt_password);
        } else {
            log::debug!("No MQTT username/password provided, connecting anonymously");
        }

        let mut writebuf = [0; 1024];
        let mut readbuf = [0; 1024];
        let mut client =
            MqttClient::<_, 5, _>::new(socket, &mut writebuf, 160, &mut readbuf, 160, config);

        match client.connect_to_broker().await {
            Ok(()) => {
                log::debug!("Connected to broker");
            }
            Err(mqtt_error) => {
                if let ReasonCode::NetworkError = mqtt_error {
                    log::debug!("MQTT Network Error");
                } else {
                    log::debug!("Other MQTT Error: {:?}", mqtt_error);
                }
            }
        }
        // TODO: Handle all the unwraps
        client.send_ping().await.unwrap();

        client
            .send_message(
                &system_topic,
                generate_json_message(&unique_id, "online").as_bytes(),
                QoS1,
                false,
            )
            .await
            .unwrap();

        loop {
            let state;
            let codes;
            {
                let device = device.lock().await;
                state = device.andon_light.get_state();
                codes = device.andon_light.get_codes();
            }
            let mut codes_repr: heapless::String<100> = heapless::String::new();
            codes_repr.push_str("[").unwrap();
            for (i, code) in codes.iter().enumerate() {
                if i > 0 {
                    // Add a comma separator between elements (but not before the first one)
                    codes_repr.push_str(",").unwrap();
                }
                codes_repr.push_str(code.as_str()).unwrap(); // Add each enum as a string
            }
            codes_repr.push_str("]").unwrap();

            let mut message = heapless::String::<150>::new();
            write!(
                message,
                r#"{{"id": "{}", "device_state": "{}", "system_state": "{}", "codes": {}}}"#,
                &unique_id,
                state.0.as_str(),
                state.1.as_str(),
                codes_repr
            )
            .unwrap();
            match client
                .send_message(&device_topic, message.as_bytes(), QoS1, false)
                .await
            {
                Ok(()) => {
                    log::debug!("MQTT message sent successfully");
                }
                Err(mqtt_error) => {
                    log::debug!("Failed to send MQTT message: {:?}", mqtt_error);
                }
            }
            Timer::after(Duration::from_secs(20)).await;
        }
    }
}

pub fn generate_mqtt_topic(device_id: &heapless::String<32>, suffix: &str) -> heapless::String<40> {
    let mut topic = heapless::String::<40>::new();
    write!(topic, "andon/{}/{}", device_id, suffix).unwrap();
    topic
}

pub fn generate_json_message(
    device_id: &heapless::String<32>,
    status: &str,
) -> heapless::String<70> {
    let mut message = heapless::String::<70>::new();
    write!(
        message,
        r#"{{"status": "{}", "id": "{}"}}"#,
        status, device_id
    )
    .unwrap();
    message
}

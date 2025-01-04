#![no_std]
#![no_main]

mod prelude;
use prelude::*;

use andon_light_core::OutputSpiDevice;
use andon_light_macros::{generate_default_from_env, ErrorCodesEnum};

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::*,
    dma_buffers,
    gpio::{GpioPin, Io, Level, Output, NO_PIN},
    i2c::I2C,
    peripherals::Peripherals,
    peripherals::{I2C0, SPI2},
    prelude::*,
    spi::{
        master::{Spi, SpiDmaBus},
        FullDuplexMode, SpiMode,
    },
    system::SystemControl,
    Async,
};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use esp_hal::timer::timg::TimerGroup;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use tcs3472::Tcs3472;

const DEFAULT_LED_AMOUNT: usize = generate_default_from_env!(DEFAULT_LED_AMOUNT, 16);

#[derive(ErrorCodesEnum)]
pub enum ErrorCodes {
    #[code(message = "Error code 1", level = "warn")]
    ErrorCode1,
}

type AndonLight = andon_light_core::AndonLight<ErrorCodes, { ErrorCodes::MIN_SET_SIZE }>;

const CONFIG_BUFFER_SIZE: usize = 4096;
type Spi2Bus = Mutex<NoopRawMutex, SpiDmaBus<'static, SPI2, DmaChannel0, FullDuplexMode, Async>>;
type AndonAsyncMutex = Mutex<CriticalSectionRawMutex, AndonLight>;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Config {
    version: u32,
    core_config: CoreConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            version: 5,
            core_config: CoreConfig::default(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CoreConfig {
    leds_amount: u8,
}

impl Default for CoreConfig {
    fn default() -> Self {
        Self { leds_amount: 4 }
    }
}

#[embassy_executor::task]
async fn blink_led(mut led: Output<'static, GpioPin<4>>) {
    led.set_high();
    loop {
        Timer::after(Duration::from_millis(100)).await;
        led.toggle();
    }
}

#[embassy_executor::task]
async fn run_andon_light(
    mut device: (impl OutputSpiDevice + 'static),
    andon_light: &'static AndonAsyncMutex,
) {
    {
        let mut andon_light = andon_light.lock().await;

        // Test procedure
        andon_light.run_test_procedure(&mut device).await;
        andon_light.signal(&mut device).await;
        Timer::after(Duration::from_millis(andon_light.get_speed() as u64)).await;
    }

    // Regular operation
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
async fn run_rgb_probe(
    mut sensor: Tcs3472<I2C<'static, I2C0, Async>>,
    andon_light: &'static AndonAsyncMutex,
) {
    let mut ticker = Ticker::every(Duration::from_millis(500));

    if let Err(e) = sensor.enable().await {
        esp_println::dbg!(e);
        return;
    }

    if let Err(e) = sensor.enable_rgbc().await {
        esp_println::dbg!(e);
        return;
    }

    while !sensor.is_rgbc_status_valid().await.unwrap() {
        // wait for measurement to be available
    }

    loop {
        match sensor.read_all_channels().await {
            Err(e) => {
                esp_println::dbg!(e);
            }
            Ok(measurement) => {
                esp_println::println!(
                    "Raw values: {:.2} ({:.2}, {:.2}, {:.2}) ",
                    measurement.clear,
                    measurement.red,
                    measurement.green,
                    measurement.blue,
                );
                let (r, g, b, color) =
                    normalize_rgb(measurement.red, measurement.green, measurement.blue);
                esp_println::println!(
                    "Normalized RGB: ({:.2}, {:.2}, {:.2}) -> Basic Color: {}",
                    r,
                    g,
                    b,
                    color,
                );
                {
                    let mut andon_light = andon_light.lock().await;
                    match color {
                        "Red" | "Pink" | "Magenta" => {
                            andon_light.set_device_state(andon_light_core::DeviceState::Error);
                        }
                        "Green" | "Mint" | "Lime" => {
                            andon_light.set_device_state(andon_light_core::DeviceState::Ok);
                        }
                        "Blue" | "Violet" | "Azure" => {
                            andon_light.set_device_state(andon_light_core::DeviceState::Idle);
                        }
                        _ => {
                            andon_light.set_device_state(andon_light_core::DeviceState::Warn);
                        }
                    }
                }
            }
        }

        ticker.next().await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

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
    let leds_select = ControlPin {
        pin: Output::new(io.pins.gpio3, Level::High),
    };

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi: SpiDmaBus<SPI2, DmaChannel0, FullDuplexMode, Async> =
        Spi::new(peripherals.SPI2, 4u32.MHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN)
            .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

    let sd_select = Output::new(io.pins.gpio2, Level::High);
    let mut leds_amount: u8 = DEFAULT_LED_AMOUNT as u8;
    let mut buffer = [0u8; CONFIG_BUFFER_SIZE];

    static SPI_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    let sd_reader = SdReader::new(spi_bus, sd_select, delay);
    let result = sd_reader
        .read_config::<Config>("CONFIG.JSO", &mut buffer)
        .await;
    if let Some(config) = result {
        esp_println::println!("config read properly");
        leds_amount = config.core_config.leds_amount;
    }

    let spi_dev = SpiDev {
        device: SpiDevice::new(spi_bus, leds_select),
    };

    static ANDON: StaticCell<AndonAsyncMutex> = StaticCell::new();
    let andon_light = ANDON.init(Mutex::new(AndonLight::new(leds_amount, 10, 150)));

    spawner
        .spawn(run_andon_light(spi_dev, andon_light))
        .unwrap();

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio7,
        100.kHz(),
        &clocks,
    );
    let sensor = Tcs3472::new(i2c);
    spawner.spawn(run_rgb_probe(sensor, andon_light)).unwrap();
}

enum ColorLevel {
    High,
    Medium,
    Low,
}

// Function to determine the basic color based on normalized RGB values
fn map_to_basic_color(r: u16, g: u16, b: u16) -> &'static str {
    // Define the basic 12 colors. These are just names for simplicity.
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

fn normalize_rgb(red: u16, green: u16, blue: u16) -> (u16, u16, u16, &'static str) {
    let max = max_of_three(red, green, blue);
    esp_println::println!("{:?}", max);
    if max == 0 {
        return (0, 0, 0, "Black");
    }
    let normalize = |val: u16| (val as f32 / max as f32 * 100.0) as u16;
    let (r, g, b) = (normalize(red), normalize(green), normalize(blue));
    (r, g, b, map_to_basic_color(r, g, b))
}

pub fn max_of_three(a: u16, b: u16, c: u16) -> u16 {
    let mut max = a;
    if b > max {
        max = b;
    }
    if c > max {
        max = c;
    }
    max
}

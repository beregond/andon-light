use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_1::digital::{ErrorType, OutputPin};
use embedded_hal_async::spi::SpiDevice as _;
use esp_hal::{
    dma::*,
    gpio::{GpioPin, Output},
    peripherals::SPI2,
    spi::{master::SpiDmaBus, FullDuplexMode},
    Async,
};

pub type Spi2Bus =
    Mutex<NoopRawMutex, SpiDmaBus<'static, SPI2, DmaChannel0, FullDuplexMode, Async>>;

#[derive(Debug, Clone, Copy)]
pub struct Color {
    /// Red
    pub r: u8,
    /// Green
    pub g: u8,
    /// Blue
    pub b: u8,
}

enum DeviceState {
    Ok,    // I don't have to do anything, the system is running
    Idle,  // the system is idle, I may have to do something, but I don't have to do it now
    Warn,  // I should do something, but it's not urgent
    Error, // I have to do something now
}

enum SystemState {
    Ok,
    Warn,
    Error,
}

pub struct AndonLight {
    device_state: DeviceState,
    system_state: SystemState,
    leds_amount: u8,
}

struct ControlPin<'d, T> {
    pin: Output<'d, T>,
}

impl<'d, T> OutputPin for ControlPin<'d, T>
where
    T: esp_hal::gpio::OutputPin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }
}
impl<'d, T> ErrorType for ControlPin<'d, T> {
    type Error = core::convert::Infallible;
}

impl AndonLight {
    pub fn new(leds_amount: u8) -> Self {
        Self {
            device_state: DeviceState::Ok,
            system_state: SystemState::Ok,
            leds_amount,
        }
    }

    pub async fn run_test_procedure<'a>(
        &mut self,
        spi: &'static Spi2Bus,
        cs: Output<'static, GpioPin<2>>,
    ) {
        let pin = ControlPin { pin: cs };
        let mut dev = SpiDevice::new(spi, pin);
        let green = Color { r: 0, g: 16, b: 0 };
        let red = Color { r: 16, g: 0, b: 0 };
        let blue = Color { r: 0, g: 0, b: 16 };
        let empty = Color { r: 0, g: 0, b: 0 };
        let white = Color { r: 8, g: 8, b: 8 };

        let send = [[empty; 16], [red; 16], [green; 16], [blue; 16]];

        let send2 = [
            [empty; 16],
            [
                white, white, empty, empty, white, white, empty, empty, white, white, empty, empty,
                white, white, empty, empty,
            ],
            [
                empty, white, white, empty, empty, white, white, empty, empty, white, white, empty,
                empty, white, white, empty,
            ],
            [
                empty, empty, white, white, empty, empty, white, white, empty, empty, white, white,
                empty, empty, white, white,
            ],
        ];

        loop {
            for row in send.iter() {
                for item in row.iter() {
                    self.send_color(&mut dev, item).await;
                }
                Timer::after(Duration::from_millis(200)).await;
            }
            for row in send2.iter() {
                for item in row.iter() {
                    self.send_color(&mut dev, item).await;
                }
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }

    async fn send_color<'a>(
        &mut self,
        spi: &mut SpiDevice<
            '_,
            NoopRawMutex,
            SpiDmaBus<
                'static,
                esp_hal::peripherals::SPI2,
                esp_hal::dma::DmaChannel0,
                FullDuplexMode,
                Async,
            >,
            ControlPin<'_, GpioPin<2>>,
        >,
        color: &Color,
    ) {
        // Priming spi as it eats first byte ¯\_(ツ)_/¯
        spi.write(&[0b0]).await.unwrap();
        let data = to_bytes(color.g);
        spi.write(&data).await.unwrap();
        let data = to_bytes(color.r);
        spi.write(&data).await.unwrap();
        let data = to_bytes(color.b);
        spi.write(&data).await.unwrap();
    }
}

fn to_bytes(mut data: u8) -> [u8; 4] {
    // This function is copied from smart-leds crate.
    //
    // Send two bits in one spi byte. High time first, then the low time
    // The maximum for T0H is 500ns, the minimum for one bit 1063 ns.
    // These result in the upper and lower spi frequency limits
    let mut result = [0b0; 4];
    let patterns = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];
    for item in result.iter_mut() {
        let bits = (data & 0b1100_0000) >> 6;
        *item = patterns[bits as usize];
        data <<= 2;
    }
    result
}

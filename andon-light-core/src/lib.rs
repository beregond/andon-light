#![no_std]

use embassy_time::{Duration, Timer};

static PATTERN_DEVICE_OK: u8 = 0b1111;
static PATTERN_DEVICE_IDLE: u8 = 0b0011;
static PATTERN_DEVICE_WARN: u8 = 0b0110;
static PATTERN_DEVICE_ERROR: u8 = 0b1001;

static PATTERN_SYSTEM_OK: u8 = 0b0000;
static PATTERN_SYSTEM_WARN: u8 = 0b0001;
static PATTERN_SYSTEM_ERROR: u8 = 0b0100;

static PATTERN_FATAL_ERROR: u8 = 0b0101;

pub trait OutputSpiDevice<Word: Copy + 'static = u8> {
    type Error: core::fmt::Debug;
    async fn write(&mut self, buf: &[Word]) -> Result<(), Self::Error>;
}

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
    speed: u16,
}

impl AndonLight {
    pub fn new(leds_amount: u8, speed: u16) -> Self {
        Self {
            device_state: DeviceState::Ok,
            system_state: SystemState::Ok,
            leds_amount,
            speed,
        }
    }

    #[inline]
    fn leds_per_segment(&self) -> u8 {
        self.leds_amount / 4
    }

    pub async fn signal(&mut self, device: &mut impl OutputSpiDevice) {
        let green = Color { r: 0, g: 16, b: 0 };
        for _ in 0..self.leds_amount {
            self.send_color(device, &green).await;
        }
    }

    pub async fn run_test_procedure(&mut self, device: &mut impl OutputSpiDevice) {
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
            [empty; 16],
        ];

        for row in send.iter() {
            for color in row.iter() {
                self.send_color(device, color).await;
            }
            Timer::after(Duration::from_millis(self.speed as u64)).await;
        }
        for row in send2.iter() {
            for color in row.iter() {
                self.send_color(device, color).await;
            }
            Timer::after(Duration::from_millis(self.speed as u64)).await;
        }
    }

    async fn send_color<'a>(&mut self, spi: &mut impl OutputSpiDevice, color: &Color) {
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

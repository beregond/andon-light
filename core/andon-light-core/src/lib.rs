#![no_std]

use embassy_time::{Duration, Timer};

/// A marker for a color channel. It only speciefies if the channel is enabled in pattern
/// and is intened to be transformed later into actual color.
#[derive(Clone, Copy)]
struct Marker {
    /// Red
    pub r: bool,
    /// Green
    pub g: bool,
    /// Blue
    pub b: bool,
}
type Pattern = [Marker; 4];

impl Marker {
    pub fn as_color(&self, brightness: u8) -> Color {
        let parts = self.r as u8 + self.g as u8 + self.b as u8;
        if parts == 0 {
            return Color { r: 0, g: 0, b: 0 };
        }
        let actual_brightness = brightness / parts;
        Color {
            r: self.r as u8 * actual_brightness,
            g: self.g as u8 * actual_brightness,
            b: self.b as u8 * actual_brightness,
        }
    }
}

/// Color is full RGB representation; it can be created out of Marker
/// and is used when ouptput device is smart led
#[derive(Debug, Clone, Copy)]
pub struct Color {
    /// Red
    pub r: u8,
    /// Green
    pub g: u8,
    /// Blue
    pub b: u8,
}

static RED: Marker = Marker {
    r: true,
    g: false,
    b: false,
};
static GREEN: Marker = Marker {
    r: false,
    g: true,
    b: false,
};
static BLUE: Marker = Marker {
    r: false,
    g: false,
    b: true,
};
static YELLOW: Marker = Marker {
    r: true,
    g: true,
    b: false,
};
static CYAN: Marker = Marker {
    r: false,
    g: true,
    b: true,
};
static MAGENTA: Marker = Marker {
    r: true,
    g: false,
    b: true,
};
static WHITE: Marker = Marker {
    r: true,
    g: true,
    b: true,
};
static OFF: Marker = Marker {
    r: false,
    g: false,
    b: false,
};

macro_rules! combine_markers {
    ($left:expr, $right:expr) => {
        Marker {
            r: $left.r || $right.r,
            g: $left.g || $right.g,
            b: $left.b || $right.b,
        }
    };
}

const fn combine_patterns(first: Pattern, second: Pattern) -> Pattern {
    [
        combine_markers!(first[0], second[0]),
        combine_markers!(first[1], second[1]),
        combine_markers!(first[2], second[2]),
        combine_markers!(first[3], second[3]),
    ]
}

/// Some patterns for boot and testing
static PATTERN_OFF: Pattern = [OFF; 4];
static PATTERN_TEST_RED: Pattern = [RED; 4];
static PATTERN_TEST_GREEN: Pattern = [GREEN; 4];
static PATTERN_TEST_BLUE: Pattern = [BLUE; 4];
static PATTERN_TEST_DIRECTION_1: Pattern = [WHITE, WHITE, OFF, OFF];
static PATTERN_TEST_DIRECTION_2: Pattern = [OFF, WHITE, WHITE, OFF];
static PATTERN_TEST_DIRECTION_3: Pattern = [OFF, OFF, WHITE, WHITE];

// Pattern are notation of pattern AND the color of the LED.
// They are noted starting from "bottom" of the device_state
// (turn head to the right to see how it looks)
// Here some patterns are partial, as they are combined to create full response
static PART_DEVICE_OK: Pattern = [GREEN, GREEN, GREEN, GREEN];
static PART_DEVICE_IDLE: Pattern = [OFF, OFF, BLUE, BLUE];
static PART_DEVICE_WARN: Pattern = [OFF, YELLOW, YELLOW, OFF];

static PART_SYSTEM_OK: Pattern = [OFF, OFF, OFF, OFF];
static PART_SYSTEM_WARN: Pattern = [CYAN, OFF, OFF, OFF];

// Now there comes nine actual patterns, which mostly are combination of parts
static PATTERN_OK_OK: Pattern = combine_patterns(PART_SYSTEM_OK, PART_DEVICE_OK);
/// System warn must not show ok for device
static PATTERN_WARN_OK: Pattern = combine_patterns(PART_SYSTEM_WARN, PATTERN_OFF);
static PATTERN_OK_IDLE: Pattern = combine_patterns(PART_SYSTEM_OK, PART_DEVICE_IDLE);
static PATTERN_WARN_IDLE: Pattern = combine_patterns(PART_SYSTEM_WARN, PART_DEVICE_IDLE);
static PATTERN_WARN_WARN: Pattern = combine_patterns(PART_SYSTEM_WARN, PART_DEVICE_WARN);
static PATTERN_OK_WARN: Pattern = combine_patterns(PART_SYSTEM_OK, PART_DEVICE_WARN);
static PATTERN_DEVICE_ERROR: Pattern = [RED, OFF, OFF, RED];
static PATTERN_SYSTEM_ERROR: Pattern = [OFF, MAGENTA, OFF, OFF];

/// This pattern is the only exceptional one, as it's used for most rare case (hopefully) - when both system and device are in error state
static PATTERN_FATAL_ERROR: Pattern = [OFF, MAGENTA, OFF, RED];

pub trait OutputSpiDevice<Word: Copy + 'static = u8> {
    type Error: core::fmt::Debug;
    #[allow(async_fn_in_trait)]
    async fn write(&mut self, buf: &[Word]) -> Result<(), Self::Error>;
}

pub enum DeviceState {
    Ok,    // I don't have to do anything, the system is running
    Idle,  // the system is idle, I may have to do something, but I don't have to do it now
    Warn,  // I should do something, but it's not urgent
    Error, // I have to do something now
}

pub enum SystemState {
    Ok,
    Warn,
    Error,
}

#[derive(Debug)]
pub enum ErrorType {
    DeviceIdle,
    DeviceWarn,
    DeviceError,
    SystemWarn,
    SystemError,
}

enum Scaling {
    Stretch,
    Repeat,
}

fn collapse(system: &SystemState, device: &DeviceState) -> Pattern {
    match (system, device) {
        (SystemState::Ok, DeviceState::Ok) => PATTERN_OK_OK,
        (SystemState::Warn, DeviceState::Ok) => PATTERN_WARN_OK,
        (SystemState::Ok, DeviceState::Idle) => PATTERN_OK_IDLE,
        (SystemState::Warn, DeviceState::Idle) => PATTERN_WARN_IDLE,
        (SystemState::Warn, DeviceState::Warn) => PATTERN_WARN_WARN,
        (SystemState::Ok, DeviceState::Warn) => PATTERN_OK_WARN,

        (SystemState::Error, DeviceState::Error) => PATTERN_FATAL_ERROR,
        (SystemState::Error, _) => PATTERN_SYSTEM_ERROR,
        (_, DeviceState::Error) => PATTERN_DEVICE_ERROR,
    }
}

pub trait ErrorCodesBase: Sized + core::fmt::Debug + Eq + PartialEq + core::hash::Hash {
    const SIZE: usize;
    const MIN_SET_SIZE: usize;

    fn as_str(&self) -> &'static str;
    fn from_str(value: &str) -> Result<Self, &'static str>;
    fn description(&self) -> &'static str;
    fn level(&self) -> ErrorType;
}

pub struct AndonLight<T: ErrorCodesBase, const U: usize> {
    leds_amount: u8,
    brightness: u8,
    speed: u16,
    codes: heapless::FnvIndexSet<T, U>,
}

impl<T: ErrorCodesBase, const U: usize> AndonLight<T, U> {
    pub const fn new(leds_amount: u8, brightness: u8, speed: u16) -> Self {
        Self {
            codes: heapless::FnvIndexSet::<T, U>::new(),
            leds_amount,
            brightness,
            speed,
        }
    }

    pub fn notify(&mut self, code: T) {
        self.codes.insert(code).unwrap();
    }

    pub fn resolve(&mut self, code: T) {
        self.codes.remove(&code);
    }

    pub fn get_speed(&self) -> u16 {
        self.speed
    }

    #[inline]
    fn leds_per_segment(&self) -> u8 {
        self.leds_amount / 4
    }

    async fn draw_pattern(
        &mut self,
        device: &mut impl OutputSpiDevice,
        pattern: Pattern,
        scaling: Scaling,
    ) {
        // TODO: Implement this better
        let colors = [
            pattern[0].as_color(self.brightness),
            pattern[1].as_color(self.brightness),
            pattern[2].as_color(self.brightness),
            pattern[3].as_color(self.brightness),
        ];
        match scaling {
            Scaling::Stretch => {
                for color in &colors {
                    for _ in 0..self.leds_per_segment() {
                        self.send_color(device, color).await;
                    }
                }
            }
            Scaling::Repeat => {
                for _ in 0..self.leds_per_segment() {
                    for color in &colors {
                        self.send_color(device, color).await;
                    }
                }
            }
        }
    }

    fn calculate_state(&self) -> (SystemState, DeviceState) {
        let mut system_error_counter = 0;
        let mut system_warn_counter = 0;
        let mut device_error_counter = 0;
        let mut device_warn_counter = 0;
        let mut device_idle_counter = 0;

        for code in &self.codes {
            match code.level() {
                ErrorType::DeviceIdle => device_idle_counter += 1,
                ErrorType::DeviceWarn => device_warn_counter += 1,
                ErrorType::DeviceError => device_error_counter += 1,
                ErrorType::SystemWarn => system_warn_counter += 1,
                ErrorType::SystemError => system_error_counter += 1,
            }
        }
        let device_state = if device_error_counter > 0 {
            DeviceState::Error
        } else if device_warn_counter > 0 {
            DeviceState::Warn
        } else if device_idle_counter > 0 {
            DeviceState::Idle
        } else {
            DeviceState::Ok
        };
        let system_state = if system_error_counter > 0 {
            SystemState::Error
        } else if system_warn_counter > 0 {
            SystemState::Warn
        } else {
            SystemState::Ok
        };
        (system_state, device_state)
    }

    pub async fn signal(&mut self, device: &mut impl OutputSpiDevice) {
        let (system_state, device_state) = self.calculate_state();
        self.draw_pattern(
            device,
            collapse(&system_state, &device_state),
            Scaling::Stretch,
        )
        .await;
    }

    pub async fn run_test_procedure(&mut self, device: &mut impl OutputSpiDevice) {
        let test_procedure = [
            PATTERN_OFF,
            PATTERN_TEST_RED,
            PATTERN_TEST_GREEN,
            PATTERN_TEST_BLUE,
            PATTERN_OFF,
            PATTERN_TEST_DIRECTION_1,
            PATTERN_TEST_DIRECTION_2,
            PATTERN_TEST_DIRECTION_3,
            PATTERN_OFF,
        ];
        for pattern in test_procedure {
            self.draw_pattern(device, pattern, Scaling::Repeat).await;
            Timer::after(Duration::from_millis(self.speed as u64)).await;
        }
    }

    async fn send_color(&mut self, spi: &mut impl OutputSpiDevice, color: &Color) {
        // Priming spi as leds eat first byte ¯\_(ツ)_/¯
        spi.write(&[0b0]).await.unwrap();
        // ...and displaying actual colors
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

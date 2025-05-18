#![no_std]

pub mod colors;

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
// TODO: rename it to something more appropriate
#[derive(Debug, Clone, Copy)]
struct Color {
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
// System warn must not show 'ok state' for device
static PATTERN_WARN_OK: Pattern = combine_patterns(PART_SYSTEM_WARN, PATTERN_OFF);
static PATTERN_OK_IDLE: Pattern = combine_patterns(PART_SYSTEM_OK, PART_DEVICE_IDLE);
static PATTERN_WARN_IDLE: Pattern = combine_patterns(PART_SYSTEM_WARN, PART_DEVICE_IDLE);
static PATTERN_WARN_WARN: Pattern = combine_patterns(PART_SYSTEM_WARN, PART_DEVICE_WARN);
static PATTERN_OK_WARN: Pattern = combine_patterns(PART_SYSTEM_OK, PART_DEVICE_WARN);
static PATTERN_DEVICE_ERROR: Pattern = [RED, OFF, OFF, RED];
static PATTERN_SYSTEM_ERROR: Pattern = [OFF, MAGENTA, OFF, OFF];

// This pattern is the only exceptional one, therefore it's used for (hopefully) most rare case
// - when both system and device are in error state
static PATTERN_FATAL_ERROR: Pattern = [OFF, MAGENTA, OFF, RED];

pub enum DeviceState {
    Ok,    // I don't have to do anything, the system is running
    Idle,  // the system is idle, I may have to do something, but I don't have to do it now
    Warn,  // I should do something, but it's not urgent
    Error, // I have to do something now
}

pub enum SystemState {
    Ok,    // Everything is ok
    Warn,  // There is minor problem, that does not prevent the system from working
    Error, // There is a problem that prevents the system from working
}

// Combination of system and device states
#[derive(Debug)]
pub enum ErrorType {
    Ok,
    DeviceIdle,
    DeviceWarn,
    DeviceError,
    SystemWarn,
    SystemError,
}

// How annoying alert you should set up.
// Since the andon state is mix of many warns and states - you may want to simply
// know how annoying the alert (like buzzer) should be. Check methot that maps
// states to that enum to see how it works.
#[derive(Debug, Eq, PartialEq)]
pub enum AlertLevel {
    Chill,     // Everything is ok
    Attentive, // There is something to be aware of, but of low priority
    Important, // There is important problem, that should be solved ASAP
    Urgent,    // Shit just hit the fan 💩
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
    const MIN_SET_SIZE: usize;

    fn as_str(&self) -> &'static str;
    fn from_str(value: &str) -> Result<Self, &'static str>;
    fn description(&self) -> &'static str;
    fn level(&self) -> ErrorType;
}

/// Structure that represents Andon light
///
/// For BUFFER_SIZE please provide max amount of supported leds multiplied by 12 - in the future it
/// will be solved by "complex generic constants" in Rust, but for now it is no available yet.
pub struct AndonLight<T: ErrorCodesBase, const U: usize, const BUFFER_SIZE: usize> {
    leds_amount: u8,
    brightness: u8,
    speed: u16,
    codes: heapless::FnvIndexSet<T, U>,
}

impl<T: ErrorCodesBase, const U: usize, const BUFFER_SIZE: usize> AndonLight<T, U, BUFFER_SIZE> {
    pub const fn new(leds_amount: u8, brightness: u8, speed: u16) -> Self {
        Self {
            codes: heapless::FnvIndexSet::<T, U>::new(),
            leds_amount,
            brightness,
            speed,
        }
    }

    pub fn notify(&mut self, code: T) -> bool {
        log::debug!("Notify: {} - {}", code.as_str(), code.description());
        if let ErrorType::Ok = code.level() {
            return false;
        }
        if log::log_enabled!(log::Level::Debug) {
            let code_repr = code.as_str();
            let inserted = self.codes.insert(code).unwrap();
            if inserted {
                log::debug!("Code added {}", code_repr);
            }
            inserted
        } else {
            self.codes.insert(code).unwrap()
        }
    }

    pub fn resolve(&mut self, code: T) -> bool {
        if self.codes.contains(&code) {
            let removed = self.codes.remove(&code);
            if removed {
                log::debug!("Resolving: {}", code.as_str());
            }
            removed
        } else {
            false
        }
    }

    pub fn notify_exclusive<const Y: usize>(&mut self, code: T, exclusive: &heapless::Vec<T, Y>) {
        for item in exclusive {
            if *item == code {
                continue;
            }
            self.codes.remove(item);
        }
        if let ErrorType::Ok = code.level() {
            return;
        }
        self.codes.insert(code).unwrap();
    }

    pub fn resolve_all<const Y: usize>(&mut self, codes: &heapless::Vec<T, Y>) {
        for code in codes {
            self.codes.remove(code);
        }
    }

    pub fn get_speed(&self) -> u16 {
        self.speed
    }

    #[inline]
    fn leds_per_segment(&self) -> u8 {
        self.leds_amount / 4
    }

    pub fn generate_signal(&mut self) -> heapless::Vec<u8, BUFFER_SIZE> {
        let (system_state, device_state) = self.calculate_state();
        self.generate_pattern(collapse(&system_state, &device_state), Scaling::Stretch)
    }

    fn generate_pattern(
        &mut self,
        pattern: Pattern,
        scaling: Scaling,
    ) -> heapless::Vec<u8, BUFFER_SIZE> {
        // TODO: allow reversed pattern
        let colors: [Color; 4] = core::array::from_fn(|i| pattern[i].as_color(self.brightness));
        let mut buffer = heapless::Vec::<u8, BUFFER_SIZE>::new();
        match scaling {
            Scaling::Stretch => {
                for color in &colors {
                    for _ in 0..self.leds_per_segment() {
                        buffer.extend_from_slice(&translate_color(color)).unwrap();
                    }
                }
            }
            Scaling::Repeat => {
                for _ in 0..self.leds_per_segment() {
                    for color in &colors {
                        buffer.extend_from_slice(&translate_color(color)).unwrap();
                    }
                }
            }
        }

        buffer
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
                ErrorType::Ok => {}
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

    pub fn calculate_alert_level(&self) -> AlertLevel {
        let (system_state, device_state) = self.calculate_state();
        match (system_state, device_state) {
            (SystemState::Ok, DeviceState::Ok) => AlertLevel::Chill,
            (SystemState::Warn, DeviceState::Ok) => AlertLevel::Attentive,
            (SystemState::Ok, DeviceState::Idle) => AlertLevel::Attentive,
            (SystemState::Warn, DeviceState::Idle) => AlertLevel::Attentive,
            (SystemState::Ok, DeviceState::Warn) => AlertLevel::Important,
            (SystemState::Warn, DeviceState::Warn) => AlertLevel::Important,
            (SystemState::Error, _) => AlertLevel::Urgent,
            (_, DeviceState::Error) => AlertLevel::Urgent,
        }
    }

    pub fn generate_test_patterns(&mut self) -> [heapless::Vec<u8, BUFFER_SIZE>; 9] {
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

        core::array::from_fn(|i| self.generate_pattern(test_procedure[i], Scaling::Repeat))
    }
}

// Shoutout to smart-leds crate from which big part of this code is copied
fn translate_color(color: &Color) -> [u8; 12] {
    let mut frame = [0u8; 12];
    let patterns = [0b1000_1000, 0b1000_1110, 0b1110_1000, 0b1110_1110];

    for (i, component) in [color.g, color.r, color.b].iter().enumerate() {
        let mut data = *component;
        for j in 0..4 {
            let bits = (data & 0b1100_0000) >> 6;
            frame[i * 4 + j] = patterns[bits as usize];
            data <<= 2;
        }
    }

    frame
}

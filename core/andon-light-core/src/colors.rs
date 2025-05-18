#[derive(Debug, Clone, PartialEq, Eq, Copy)]
pub enum Color {
    Magenta,
    Pink,
    Red,
    Orange,
    Yellow,
    Lime,
    Green,
    Mint,
    Cyan,
    Azure,
    Blue,
    Violet,
    Gray,
    White,
    Black,
}

#[derive(Debug)]
enum ColorLevel {
    High,
    Medium,
    Low,
}

pub struct ColorMapper {
    activation_level: u16,
    saturation_level: u16,
    // TODO: name it percentage and limit to 40
    margin: u16,
}

impl ColorMapper {
    pub fn new(activation_level: u16, saturation_level: u16, margin: u16) -> Self {
        Self {
            activation_level,
            saturation_level,
            margin,
        }
    }
    pub fn translate_proportionally(&self, red: u16, green: u16, blue: u16) -> Color {
        if red + green + blue <= self.activation_level {
            return Color::Black;
        }
        normalize_rgb(red, green, blue, self.saturation_level, self.margin)
    }
}

impl Default for ColorMapper {
    fn default() -> Self {
        Self {
            activation_level: 100,
            saturation_level: 1000,
            margin: 10,
        }
    }
}

fn normalize_rgb(red: u16, green: u16, blue: u16, saturation_level: u16, margin: u16) -> Color {
    let max = red.max(blue).max(green);
    if max == 0 {
        return Color::Black;
    }
    let normalize = |val: u16| (val as f32 / max as f32 * 100.0) as u16;
    let (r, g, b) = (normalize(red), normalize(green), normalize(blue));
    match (
        map_to_level(r, margin),
        map_to_level(g, margin),
        map_to_level(b, margin),
    ) {
        (ColorLevel::High, ColorLevel::Low, ColorLevel::High) => Color::Magenta,
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Medium) => Color::Pink,
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Low) => Color::Red,
        (ColorLevel::High, ColorLevel::Medium, ColorLevel::Low) => Color::Orange,
        (ColorLevel::High, ColorLevel::High, ColorLevel::Low) => Color::Yellow,
        (ColorLevel::Medium, ColorLevel::High, ColorLevel::Low) => Color::Lime,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Low) => Color::Green,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Medium) => Color::Mint,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::High) => Color::Cyan,
        (ColorLevel::Low, ColorLevel::Medium, ColorLevel::High) => Color::Azure,
        (ColorLevel::Low, ColorLevel::Low, ColorLevel::High) => Color::Blue,
        (ColorLevel::Medium, ColorLevel::Low, ColorLevel::High) => Color::Violet,
        (ColorLevel::High, ColorLevel::High, ColorLevel::High) => {
            // Special case when summation of all colors is not too high
            if red + green + blue < saturation_level {
                Color::Gray
            } else {
                Color::White
            }
        }
        (x, y, z) => {
            panic!(
                "Unexpected color mapping: ({}, {}, {}) - ({:?}, {:?}, {:?})",
                r, g, b, x, y, z
            )
        }
    }
}

#[inline]
fn map_to_level(value: u16, margin: u16) -> ColorLevel {
    if value > 100 - margin {
        ColorLevel::High
    } else if value > 60 - margin {
        ColorLevel::Medium
    } else {
        ColorLevel::Low
    }
}

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
    margin_percent_points: u16,
}

impl ColorMapper {
    pub fn new(activation_level: u16, saturation_level: u16, margin_percent_points: u16) -> Self {
        Self {
            activation_level,
            saturation_level,
            margin_percent_points,
        }
    }
    pub fn translate_proportionally(&self, red: u16, green: u16, blue: u16) -> Color {
        if red + green + blue <= self.activation_level {
            return Color::Black;
        }
        normalize_rgb(
            red,
            green,
            blue,
            self.saturation_level,
            self.margin_percent_points,
        )
    }
}

impl Default for ColorMapper {
    fn default() -> Self {
        Self::new(100, 1000, 10)
    }
}

// TODO: document this
fn normalize_rgb(
    red: u16,
    green: u16,
    blue: u16,
    saturation_level: u16,
    margin_percent_points: u16,
) -> Color {
    let max = red.max(blue).max(green);
    if max == 0 {
        return Color::Black;
    }
    let normalize = |val: u16| (val as f32 / max as f32 * 100.0) as u16;
    let (r, g, b) = (normalize(red), normalize(green), normalize(blue));
    match (
        map_to_level(r, margin_percent_points),
        map_to_level(g, margin_percent_points),
        map_to_level(b, margin_percent_points),
    ) {
        (ColorLevel::High, ColorLevel::Low, ColorLevel::High) => Color::Magenta,
        (ColorLevel::High, ColorLevel::Medium, ColorLevel::High) => Color::Magenta,
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Medium) => Color::Pink,
        (ColorLevel::High, ColorLevel::Low, ColorLevel::Low) => Color::Red,
        (ColorLevel::High, ColorLevel::Medium, ColorLevel::Medium) => Color::Red,
        (ColorLevel::High, ColorLevel::Medium, ColorLevel::Low) => Color::Orange,
        (ColorLevel::High, ColorLevel::High, ColorLevel::Low) => Color::Yellow,
        (ColorLevel::High, ColorLevel::High, ColorLevel::Medium) => Color::Yellow,
        (ColorLevel::Medium, ColorLevel::High, ColorLevel::Low) => Color::Lime,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Low) => Color::Green,
        (ColorLevel::Medium, ColorLevel::High, ColorLevel::Medium) => Color::Green,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::Medium) => Color::Mint,
        (ColorLevel::Low, ColorLevel::High, ColorLevel::High) => Color::Cyan,
        (ColorLevel::Medium, ColorLevel::High, ColorLevel::High) => Color::Cyan,
        (ColorLevel::Low, ColorLevel::Medium, ColorLevel::High) => Color::Azure,
        (ColorLevel::Low, ColorLevel::Low, ColorLevel::High) => Color::Blue,
        (ColorLevel::Medium, ColorLevel::Medium, ColorLevel::High) => Color::Blue,
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
    if value >= 100 - margin.min(100) {
        ColorLevel::High
    } else if value >= 60 - margin.min(60) {
        ColorLevel::Medium
    } else {
        ColorLevel::Low
    }
}

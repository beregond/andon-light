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

enum ColorLevel {
    High,
    Medium,
    Low,
}

pub fn translate_color_proportionally(
    red: u16,
    green: u16,
    blue: u16,
    activation_level: u16,
    saturation_level: u16,
) -> Color {
    if red + green + blue <= activation_level {
        return Color::Black;
    }
    normalize_rgb(red, green, blue, saturation_level)
}

fn normalize_rgb(red: u16, green: u16, blue: u16, saturation_level: u16) -> Color {
    let max = red.max(blue).max(green);
    if max == 0 {
        return Color::Black;
    }
    let normalize = |val: u16| (val as f32 / max as f32 * 100.0) as u16;
    let (r, g, b) = (normalize(red), normalize(green), normalize(blue));
    match map_three(r, g, b) {
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
        (_, _, _) => {
            panic!("Unexpected color mapping: ({}, {}, {})", r, g, b)
        }
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

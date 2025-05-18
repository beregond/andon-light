use andon_light_core::colors::{translate_color_proportionally, Color};

#[test]
fn test_simple_colors_translation() {
    assert_eq!(
        translate_color_proportionally(100, 0, 0, 20, 80),
        Color::Red
    );
    assert_eq!(
        translate_color_proportionally(0, 100, 0, 20, 80),
        Color::Green
    );
    assert_eq!(
        translate_color_proportionally(0, 0, 100, 20, 80),
        Color::Blue
    );
    assert_eq!(
        translate_color_proportionally(0, 100, 100, 20, 80),
        Color::Cyan
    );
    assert_eq!(
        translate_color_proportionally(100, 100, 0, 20, 80),
        Color::Yellow
    );
    assert_eq!(
        translate_color_proportionally(100, 0, 100, 20, 80),
        Color::Magenta
    );
    assert_eq!(
        translate_color_proportionally(100, 70, 0, 20, 80),
        Color::Orange
    );
    assert_eq!(
        translate_color_proportionally(100, 0, 70, 20, 80),
        Color::Pink
    );
    assert_eq!(
        translate_color_proportionally(0, 100, 70, 20, 80),
        Color::Mint
    );
    assert_eq!(
        translate_color_proportionally(70, 100, 0, 20, 80),
        Color::Lime
    );
    assert_eq!(
        translate_color_proportionally(70, 0, 100, 20, 80),
        Color::Violet
    );
    assert_eq!(
        translate_color_proportionally(0, 70, 100, 20, 80),
        Color::Azure
    );
}

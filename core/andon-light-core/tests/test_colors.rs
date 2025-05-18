use andon_light_core::colors::{Color, ColorMapper};

#[test]
fn test_saturation_levels() {
    let mapper = ColorMapper::new(20, 80, 10);
    assert_eq!(mapper.translate_proportionally(5, 5, 5), Color::Black);
    assert_eq!(mapper.translate_proportionally(15, 15, 15), Color::Gray);
    assert_eq!(mapper.translate_proportionally(50, 50, 50), Color::White);
}

#[test]
fn test_simple_translation() {
    let mapper = ColorMapper::new(20, 80, 10);
    assert_eq!(mapper.translate_proportionally(100, 0, 0), Color::Red);
    assert_eq!(mapper.translate_proportionally(100, 70, 70), Color::Red);
    assert_eq!(mapper.translate_proportionally(0, 100, 0), Color::Green);
    assert_eq!(mapper.translate_proportionally(70, 100, 70), Color::Green);
    assert_eq!(mapper.translate_proportionally(0, 0, 100), Color::Blue);
    assert_eq!(mapper.translate_proportionally(70, 70, 100), Color::Blue);
    assert_eq!(mapper.translate_proportionally(0, 100, 100), Color::Cyan);
    assert_eq!(mapper.translate_proportionally(70, 100, 100), Color::Cyan);
    assert_eq!(mapper.translate_proportionally(100, 100, 0), Color::Yellow);
    assert_eq!(mapper.translate_proportionally(100, 100, 70), Color::Yellow);
    assert_eq!(mapper.translate_proportionally(100, 0, 100), Color::Magenta);
    assert_eq!(
        mapper.translate_proportionally(100, 70, 100),
        Color::Magenta
    );
    assert_eq!(mapper.translate_proportionally(100, 70, 0), Color::Orange);
    assert_eq!(mapper.translate_proportionally(100, 0, 70), Color::Pink);
    assert_eq!(mapper.translate_proportionally(0, 100, 70), Color::Mint);
    assert_eq!(mapper.translate_proportionally(70, 100, 0), Color::Lime);
    assert_eq!(mapper.translate_proportionally(70, 0, 100), Color::Violet);
    assert_eq!(mapper.translate_proportionally(0, 70, 100), Color::Azure);
}

# ())))) Andon Light

Signal state with andon light!

## Requirements

This soft is dedicated to ESP32C3 and form factor assumes Seed Studio XIAO ESP32C3 board.

## Development

Check `devbin` dir to see available helpful commands

## Parts list

- ESP32C3 (from Seed Studio provided witn antenna)
- Prototype board - 2x8 cm
- 3x stripe of 16 smart leds (WS2812B) - with densito of 96 leds per meter
- SD card module (WH125 spi sd reader module)
- SD card
- USB-C cables:
  - one with angle plug for programming - because of lack of space
  - second one with straight plug for power supply
- USB charger
- 3x 2k Ohm resistors
- 2x PNP transistors (S8550)
- buzzer with generator - THT 12mm
- red LED
- USB-C THT socket - like USB4085-GF-A from GCT
- TCS3472 sensor (the one used in the project is from Adafruit)
- four pin socket and connector
- some 0.5mm^2 wire - it should be quite thin to make your life easier; you can harvest some from ethernet cable

You will also need few 3D printed parts, to print them follow [this guide](3d-models/readme.md).

Other parts:

- transparent plexiglass tube:
  - 20 mm outer diameter
  - 1 mm thickness
  - 18 mm inner diameter
  - 165 mm length
- toothpick

To mount device first without soldering - also breadboard plus jumper wires are needed.

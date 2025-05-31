# ())))) Andon Light

Signal state with andon light!

## How does it work?

Andon light is a tool for monitoring external device and signaling its state using lights and sounds.

> [!NOTE]
> Currently only configuration for TCS3472 sensor and Prusa mk4 is available. More configurations will be (hopefully) added in the future.

There are two basic components:

- **Device** - something we want to monitor, like 3d printer; state of device is read by sensor
- **System** - andon light itself.

Both of them have their statuses and below you can see how those statuses are mapped to patterns, colors and sounds.

![Patterns mapping](docs/map.png)

The algorithm works like this:

- Take the highest corresponding alert level from the sensor and system state
- Using patterns from that alert level, choose:
  - Pattern that matches both of the states
  - If no such pattern exists, choose the one that matches either of the states
- Choose sound that matches alert level

## Parts list

- To see the list of parts needed to build this project, check [this file](docs/parts.md).
- You will need few 3D printed parts, to print them follow [this guide](3d-models/readme.md).

## Development

Check `devbin` dir to see available helpful commands

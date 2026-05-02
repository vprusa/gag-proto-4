# GAG Glove Mouse Controller

This project is an upgraded wearable gesture glove built around a TTGO / T-Display class ESP32 target, multiple IMU sensors, and a custom gesture-recognition pipeline. It turns hand and finger motion into recognized gestures, visual feedback on the onboard display, and mouse-style input actions.

It is intended for:

- experimenting with glove-based human-computer interaction
- recognizing hand poses and motion patterns from multiple IMUs
- driving cursor, click, and mode-switch actions from wearable hardware
- calibrating, visualizing, and testing a multi-sensor glove stack on-device and on a desktop host

The current codebase combines upgraded hardware support with substantial software extensions, refactoring, and tuning. Compared with the original prototype lineage, this version expands the recognizer, improves offset handling and visualization, supports Linux-host unit tests, and adds practical gesture-driven mouse emulation workflows.

## What The Project Contains

- `sketch_oct12a.ino`: main firmware sketch for the glove
- `GagRecogMerged.h`: merged gesture recognizer implementation
- `GagOffsetsMerged.h`: offset storage and calibration helpers
- `GagTtgoVizMerged.h`: TTGO / T-Display visualization layer
- `tests/`: host-side tests for recognizer and offset logic
- `config.h`: feature flags and hardware/software behavior switches
- `notes/README.md`: lower-level package notes and implementation details

## Main Capabilities

- multi-IMU glove input for wrist and fingers
- pose and motion gesture recognition
- drift-reset and offset-calibration support
- on-device hand / sensor visualization
- recognized-gesture logging on the display
- gesture-triggered mouse actions and wrist-mouse toggling
- desktop-testable recognition logic

## Media Placeholders

Add project media here when available:

- `docs/media/glove-photo-overview.jpg`
  Physical glove overview photo
- `docs/media/glove-photo-closeup.jpg`
  Close-up photo of sensors, routing, and mounting
- `docs/media/glove-wire-schema.png`
  Wiring schematic / connection diagram of the glove
- `docs/media/glove-gesture-demo.gif`
  GIF recording of the glove operating on its own
- `docs/media/glove-mouse-demo.gif`
  GIF recording of the glove emulating mouse input

Suggested README embed section once the assets exist:

```md
## Photos And Demos

![Physical glove overview](docs/media/glove-photo-overview.jpg)
![Glove wiring schematic](docs/media/glove-wire-schema.png)
![Standalone glove demo](docs/media/glove-gesture-demo.gif)
![Mouse emulation demo](docs/media/glove-mouse-demo.gif)
```

## Project Background

This project is based on the upstream work:

- `https://github.com/vprusa/gag`
- `https://github.com/vprusa/gag-web`

This repository extends that foundation with upgraded hardware assumptions, broader software capabilities, and many implementation improvements developed with LLM-assisted iteration and substantial manual refinement.

In practical terms, this project should be understood as:

- a hardware-upgraded derivative of the original GAG glove work
- a software-extended and reworked firmware branch
- an experiment platform for improved gesture recognition, calibration, visualization, and mouse emulation

## Development Notes

For lower-level implementation notes, packaging details, and merged-header structure, see [notes/README.md](notes/README.md).

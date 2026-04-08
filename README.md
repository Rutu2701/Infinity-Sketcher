<div align="center">

# 🖊️ Infinity-Sketcher

**A differential-drive robot that draws any image on an unbounded canvas.**

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue?logo=python)](https://www.python.org/)
[![Arduino](https://img.shields.io/badge/Arduino-Uno%2FNano-00979D?logo=arduino)](https://www.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

<!-- BANNER IMAGE -->
<!-- Replace the placeholder below with a photo or GIF of the robot in action -->
<!-- ![Infinity-Sketcher in action](docs/images/banner.gif) -->

```
┌─────────────────────────────────────────┐
│         [ Robot Photo / GIF ]           │
│   Place your banner image here          │
│   Recommended: 1200 × 400 px           │
└─────────────────────────────────────────┘
```

</div>

---

## 📋 Table of Contents

- [Problem Statement](#-problem-statement)
- [How It Works](#-how-it-works)
- [System Architecture](#-system-architecture)
- [Hardware](#-hardware)
- [Software Pipeline](#-software-pipeline)
- [Gallery](#-gallery)
- [Quick Start](#-quick-start)
- [Configuration](#-configuration)
- [File Structure](#-file-structure)
- [Troubleshooting](#-troubleshooting)
- [Future Work](#-future-work)

---

## 🎯 Problem Statement

Most consumer plotters and drawing robots are constrained to a fixed paper size. Mounting paper precisely, keeping it taut, and dealing with mechanical frame limits are all common bottlenecks that restrict creative freedom.

**Infinity-Sketcher** removes those constraints entirely. Instead of moving the pen over a stationary sheet, the robot *drives across* any flat surface—whiteboard, floor, craft paper roll—while simultaneously lowering a pen to trace the image. The result is a mobile drawing machine with a theoretically **unbounded canvas**.

### Goals

| Goal | Description |
|------|-------------|
| **Any image** | Accept arbitrary raster images and convert them to robot paths |
| **Any surface** | Drive on whiteboards, large paper rolls, floors, and more |
| **Accurate lines** | Arc-aware speed modulation for smooth curves |
| **Open hardware** | Built entirely from off-the-shelf parts and open-source code |

---

## ⚙️ How It Works

Infinity-Sketcher combines a **Python GUI** (running on your computer) with **Arduino firmware** (running on the robot):

```
Your Computer                          Robot (Arduino)
─────────────────                      ───────────────────────
 Load Image                             Receive STEPS command
    │                                         │
 CLAHE + Otsu                            Set wheel speeds
    │                                         │
 Potrace / Skeleton                      Execute stepper moves
    │                                         │
 Midline Extraction                      Lift / lower pen servo
    │                                         │
 Curvature Detection                     Report OK / ERR
    │
 Arc-Aware Path Planning
    │
 Serial → USB → Arduino
```

1. **Image processing** — the GUI loads any image and binarises it using CLAHE + Otsu thresholding.
2. **Vectorisation** — strokes are traced via Potrace (preferred) or gradient-ridge midline extraction.
3. **Curvature analysis** — every stroke segment is classified as a straight line or an arc.
4. **Path planning** — step counts for left/right wheels are computed from robot kinematics.
5. **Serial transmission** — moves are sent over USB-serial to the Arduino one at a time.
6. **Motor execution** — the Arduino drives two stepper motors with arc-aware differential speed and lifts/lowers the pen servo.

---

## 🏗️ System Architecture

```
┌───────────────────────────────────────────────────────────┐
│                    Python GUI (v16.py)                    │
│                                                           │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐  │
│  │ Image Import │→ │  Processing  │→ │  Path Planner  │  │
│  │ & Preview    │  │  Pipeline    │  │  (kinematics)  │  │
│  └──────────────┘  └──────────────┘  └───────┬────────┘  │
│                                              │            │
│                                       Serial Commands     │
└──────────────────────────────────────────────┼────────────┘
                                               │ USB
                                    ┌──────────▼───────────┐
                                    │   Arduino (firmware)  │
                                    │                       │
                                    │  ┌─────┐   ┌──────┐  │
                                    │  │Servo│   │ PID  │  │
                                    │  │(pen)│   │(fwd) │  │
                                    │  └─────┘   └──────┘  │
                                    │  ┌──────────────────┐ │
                                    │  │ Stepper Drivers  │ │
                                    │  │  Left    Right   │ │
                                    │  └──────────────────┘ │
                                    └───────────────────────┘
```

---

## 🔧 Hardware

### Bill of Materials

| Component | Specification | Notes |
|-----------|--------------|-------|
| Microcontroller | Arduino Uno / Nano | ATmega328P |
| Stepper motors | NEMA 17, 200 steps/rev | One per drive wheel |
| Stepper drivers | A4988 / DRV8825 | 32 microsteps recommended |
| Servo motor | SG90 / MG90S | Pen lift mechanism |
| Wheels | ~60 mm diameter | Adjust in software |
| Chassis | Custom / 3-D printed | Differential drive layout |
| Power supply | 12 V, 2 A | Motors + logic |
| USB cable | A to B / Micro | Python ↔ Arduino |

### Wiring Diagram

<!-- WIRING DIAGRAM IMAGE -->
<!-- Replace the placeholder below with your actual wiring diagram -->
<!-- ![Wiring Diagram](docs/images/wiring.png) -->

```
┌─────────────────────────────────────────┐
│          [ Wiring Diagram ]             │
│   Place your wiring diagram image here  │
│   Recommended: 800 × 600 px            │
└─────────────────────────────────────────┘
```

#### Pin Assignment (Arduino)

| Pin | Signal | Description |
|-----|--------|-------------|
| 2 | STEP_L | Left stepper pulse |
| 3 | DIR_L | Left stepper direction |
| 6 | STEP_R | Right stepper pulse |
| 7 | DIR_R | Right stepper direction |
| 8 | EN_ALL | Enable (active LOW, shared) |
| 9 | FLT_L | Left driver fault (INPUT_PULLUP) |
| 10 | FLT_R | Right driver fault (INPUT_PULLUP) |
| 11 | SERVO | Pen servo signal |

---

## 🖥️ Software Pipeline

### Image Processing

```
Raw Image
   │
   ▼
CLAHE (contrast enhancement)
   │
   ▼
Gaussian Blur
   │
   ▼
Otsu Binarisation
   │
   ├──► Potrace Vectorisation (preferred)
   │         │
   │         ▼
   │    SVG Paths → Bezier Curves
   │
   └──► Gradient Ridge Midline (fallback)
             │
             ▼
        Distance Transform → Ridge Detection → Skeleton
   │
   ▼
Stroke Contours (list of polylines)
```

### Arc-Aware Path Planning

```
Stroke Contours
   │
   ▼
Curvature at Each Point  κ = |dθ/ds|
   │
   ├── κ < threshold  →  'line' segment
   └── κ ≥ threshold  →  'arc'  segment
         │
         ▼
      Differential speed:
        outer wheel = normal speed
        inner wheel = arc speed (ASPEED)
   │
   ▼
STEPS L:<n> R:<n> commands → Arduino
```

### Serial Protocol

Commands are plain-text, one per line:

| Command | Example | Description |
|---------|---------|-------------|
| `STEPS L:<n> R:<n>` | `STEPS L:100 R:95` | Move wheels (signed steps) |
| `SPEED <µs>` | `SPEED 300` | Draw speed (step delay) |
| `TSPEED <µs>` | `TSPEED 120` | Travel speed (pen up) |
| `ASPEED <µs>` | `ASPEED 500` | Arc inner-wheel speed |
| `PEN DOWN` | `PEN DOWN` | Lower pen |
| `PEN UP` | `PEN UP` | Raise pen |
| `SERVO <pin> <dn°> <up°>` | `SERVO 11 30 90` | Reconfigure servo |
| `STATUS` | `STATUS` | Report current state |
| `PING` | `PING` | Connectivity check |
| `STOP` | `STOP` | Emergency stop |

Responses are always `OK` or `ERR <reason>`.

---

## 🖼️ Gallery

<!-- GALLERY IMAGES -->
<!-- Add your output images below. Suggested layout: 3 images per row -->

```
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│                  │  │                  │  │                  │
│  [ Sample Plot   │  │  [ GUI Screenshot│  │  [ Robot Photo ] │
│    Output ]      │  │    ]             │  │                  │
│                  │  │                  │  │                  │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

<!--
Example (uncomment and replace paths after adding images to docs/images/):

| Plot Output | GUI | Robot |
|:-----------:|:---:|:-----:|
| ![Plot](docs/images/plot_output.jpg) | ![GUI](docs/images/gui_screenshot.png) | ![Robot](docs/images/robot_photo.jpg) |
-->

---

## 🚀 Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/Rutu2701/Infinity-Sketcher.git
cd Infinity-Sketcher
```

### 2. Install Python dependencies

```bash
pip install -r requirements.txt
```

> **Optional:** Install [Potrace](https://potrace.sourceforge.net/) for higher-quality vectorisation.
> On Ubuntu/Debian: `sudo apt install potrace`
> On macOS: `brew install potrace`

### 3. Flash the Arduino

Open `firmware.ino` (or `v6_arc_aware.ino`) in the Arduino IDE.
Select your board (Uno/Nano) and upload.

### 4. Run the GUI

```bash
python v16.py
```

### 5. First plot walkthrough

1. **Connect** — select the correct serial port and click **Connect**.
2. **Load image** — click **Open Image** and choose any PNG/JPG.
3. **Process** — click **Process Image**; inspect the midline preview.
4. **Generate moves** — click **Generate Moves**; review step count in the log.
5. **Demo run** — click **Demo** (pen up) to watch the robot trace without drawing.
6. **Plot** — click **Plot** to draw for real.

---

## ⚙️ Configuration

### Key Parameters (GUI sliders)

| Parameter | Default | Effect |
|-----------|---------|--------|
| Wheel diameter (mm) | 60 | Affects mm-per-step conversion |
| Axle length (mm) | 120 | Affects rotation accuracy |
| Draw speed (µs) | 300 | Stepper delay while drawing |
| Travel speed (µs) | 120 | Stepper delay while pen is up |
| Arc speed (µs) | 300 | Inner-wheel delay on curves |
| Curvature threshold | 0.05 | Sensitivity of arc detection |
| Pen down angle (°) | 30 | Servo position for drawing |
| Pen up angle (°) | 90 | Servo position while travelling |
| Paper width (mm) | 150 | Canvas width boundary |
| Paper height (mm) | 150 | Canvas height boundary |

### Robot Kinematics

```
STEPS_PER_MM  = (200 steps × 32 microsteps) / (π × wheel_diameter_mm)
STEPS_PER_DEG = (π × axle_mm / 360) × STEPS_PER_MM
```

---

## 📁 File Structure

```
Infinity-Sketcher/
├── v16.py                  # Main Python GUI application
├── v6_arc_aware.ino        # Arduino firmware (arc-aware, v6)
├── firmware.ino            # Arduino firmware (stable release)
├── requirements.txt        # Python dependencies
├── INTEGRATION_GUIDE_v18.md# Detailed algorithm & tuning guide
└── README.md               # This file
```

---

## 🐛 Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot drifts on straight lines | Wheel diameters not matched | Calibrate wheel diameter in GUI |
| Curves are too wide | Inner wheel too fast | Lower ASPEED |
| Curves are too tight | Inner wheel too slow | Raise ASPEED |
| Pen drags on travel moves | Pen-up angle too low | Increase pen-up angle |
| Pen doesn't touch paper | Pen-down angle too high | Decrease pen-down angle |
| Serial timeout / no response | Wrong port or baud rate | Verify 115200 baud, correct COM port |
| Motors not moving | Enable pin or power issue | Check 12 V supply and EN_ALL wiring |
| Image output is blurry | Ridge threshold too low | Increase `ridge_threshold` multiplier |
| Too many false arcs | Curvature threshold too low | Raise curvature threshold slider |
| Potrace not found | Binary not in PATH | Install Potrace or let fallback run |

---

## 🔭 Future Work

- [ ] **Encoder feedback** — closed-loop wheel speed control
- [ ] **Per-move ASPEED** — send arc speed with every STEPS command
- [ ] **Stroke-width compensation** — adjust pen pressure via PWM
- [ ] **Web interface** — remote control and live plot monitoring
- [ ] **Multi-pen support** — automatic colour changes
- [ ] **Adaptive curvature** — auto-tune threshold per stroke
- [ ] **G-code export** — compatibility with standard CNC senders

---

## 🤝 Contributing

Pull requests are welcome! Please open an issue first to discuss major changes.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).

---

<div align="center">

Made with ❤️ by [Rutu2701](https://github.com/Rutu2701)

</div>

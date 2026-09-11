<div align="center">

# Infinity-Sketcher

**A differential-drive robot that draws images on an unbounded canvas.**

![Infinity-Sketcher vehicle](docs/images/vehicle.png)

[![Python](https://img.shields.io/badge/Python-3.7%2B-blue?logo=python)](https://www.python.org/)
[![Arduino](https://img.shields.io/badge/Arduino-Uno%2FNano-00979D?logo=arduino)](https://www.arduino.cc/)

</div>

---

## Table of Contents

- [Problem Statement](#-problem-statement)
- [How It Works](#-how-it-works)
- [System Architecture](#-system-architecture)
- [Hardware](#-hardware)
- [Software Pipeline](#-software-pipeline)
- [HTML Plotter Lab](#-html-plotter-lab)
- [Quick Start](#-quick-start)
- [Configuration](#-configuration)
- [File Structure](#-file-structure)
- [Troubleshooting](#-troubleshooting)
- [Future Work](#-future-work)

---

## Problem Statement

Most consumer plotters and drawing robots are constrained to a fixed paper size. Infinity-Sketcher removes that constraint by mounting the drawing mechanism on a differential-drive robot that can move across a large flat surface while tracing an image.

### Goals

| Goal | Description |
|------|-------------|
| **Any image** | Accept raster images and convert them to robot paths |
| **Any surface** | Drive across whiteboards, paper and other suitable flat surfaces |
| **Accurate lines** | Generate smooth, controllable differential-drive motion |
| **Open hardware** | Use accessible hardware and open-source software |

---

## How It Works

Infinity-Sketcher combines a **Python GUI** running on a computer with **Arduino firmware** running on the robot.

```text
Your Computer                         Robot (Arduino)
─────────────────                     ─────────────────────
 Load Image                            Receive STEPS command
    │                                        │
 Image Processing                       Set wheel speeds
    │                                        │
 Vector / Skeleton Tracing              Execute stepper moves
    │                                        │
 Path Planning                          Lift / lower pen
    │                                        │
 Serial → USB → Arduino                Report OK / ERR
```

1. **Image processing** — the GUI imports and processes the selected image.
2. **Vectorisation / tracing** — image features are converted into drawable paths.
3. **Path planning** — paths are converted into differential-drive wheel motion.
4. **Serial transmission** — commands are sent over USB serial.
5. **Motor execution** — the Arduino drives the two stepper motors and pen servo.

The project also contains the skeleton-path-tracing work used for extracting drawable centerline paths from images.

---

## System Architecture

```text
┌───────────────────────────────────────────────────────┐
│                    Python GUI                         │
│                                                       │
│  Image Import → Processing → Path Planning → Serial  │
└──────────────────────────────────────┬────────────────┘
                                       │ USB
                              ┌────────▼────────┐
                              │     Arduino     │
                              │                 │
                              │ Pen Servo       │
                              │ Left Stepper    │
                              │ Right Stepper   │
                              └─────────────────┘
```

---

## Hardware

Typical hardware for the robot:

| Component | Specification |
|-----------|---------------|
| Microcontroller | Arduino Uno / Nano |
| Stepper motors | NEMA 17, 200 steps/rev |
| Stepper drivers | A4988 / DRV8825 |
| Servo motor | SG90 / MG90S |
| Wheels | ~60 mm diameter |
| Chassis | Differential-drive custom chassis |
| Power supply | 12 V motor supply |

### Arduino Pin Assignment

| Pin | Signal | Description |
|-----|--------|-------------|
| 2 | STEP_L | Left stepper pulse |
| 3 | DIR_L | Left stepper direction |
| 6 | STEP_R | Right stepper pulse |
| 7 | DIR_R | Right stepper direction |
| 8 | EN_ALL | Shared driver enable |
| 9 | FLT_L | Left driver fault |
| 10 | FLT_R | Right driver fault |
| 11 | SERVO | Pen servo signal |

---

## Software Pipeline

```text
Raw Image
   │
   ▼
Pre-processing / Thresholding
   │
   ├──► Vectorisation
   │
   └──► Skeleton / Midline tracing
             │
             ▼
        Drawable paths
             │
             ▼
       Path planning
             │
             ▼
     STEPS L:<n> R:<n>
             │
             ▼
          Arduino
```

The Python application uses OpenCV, NumPy, Pillow and serial communication. Potrace can be used when available for vectorisation.

---

## HTML Plotter Lab

`plotter_lab_v19.html` is a standalone browser-based laboratory for experimenting with the plotter pipeline and visualising processing/path-planning behaviour without changing the main Python application.

Open it directly in a modern browser:

```text
plotter_lab_v19.html
```

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/Rutu2701/Infinity-Sketcher.git
cd Infinity-Sketcher
```

### 2. Install Python dependencies

```bash
pip install -r requirements.txt
```

### 3. Flash the Arduino

Open `firmware.ino` in the Arduino IDE, select the appropriate Uno/Nano board and port, then upload it.

### 4. Run the GUI

```bash
python v17.py
```

### 5. Plot

1. Connect to the correct serial port.
2. Load an image.
3. Process the image and inspect the generated paths.
4. Generate/send the motion commands.
5. Use the pen-up/demo mode before the first physical plot.
6. Plot with the pen engaged.

---

## Configuration

Important parameters include wheel diameter, axle length, drawing/travel speed, curvature or path-processing thresholds, and pen-up/pen-down positions. Calibrate these values for the actual robot before relying on absolute dimensions.

### Robot Kinematics

```text
STEPS_PER_MM = (steps_per_rev × microsteps) / (π × wheel_diameter_mm)
```

For differential-drive rotation, wheel travel depends on the axle length and commanded rotation angle.

---

## File Structure

```text
Infinity-Sketcher/
├── v17.py                         # Main Python GUI/application
├── firmware.ino                   # Arduino robot firmware
├── requirements.txt               # Python dependencies
├── plotter_lab_v19.html           # Browser-based processing/plotter lab
├── docs/
│   └── images/
│       └── vehicle.png            # Project vehicle image
└── README.md                      # Project documentation
```

Only the files required for the main application, firmware, dependency installation, browser lab, project presentation, and documentation are kept on `main`. Development/debug outputs and generated test artifacts are intentionally excluded.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot drifts on straight lines | Wheel mismatch | Calibrate wheel diameter / wheel alignment |
| Motors do not move | Driver enable or power issue | Check motor supply, wiring and enable signal |
| Serial connection fails | Wrong port or baud rate | Verify the selected port and firmware baud rate |
| Pen drags while travelling | Pen-up position is incorrect | Adjust the servo position |
| Curves are inaccurate | Geometry/calibration mismatch | Calibrate axle length and wheel diameter |
| Potrace unavailable | Optional dependency missing | Install Potrace or use the available tracing path |

---

## Future Work

- [ ] Encoder feedback and closed-loop motion
- [ ] Improved curvature-aware speed control
- [ ] Stroke-width / pen-pressure compensation
- [ ] Remote web control and live plotting
- [ ] Multi-pen support
- [ ] G-code export

---

## Contributing

Pull requests are welcome. Open an issue first for substantial changes so the implementation can be discussed before integration.

---

## License

This project is licensed under the MIT License.

---

<div align="center">

Made by [Rutu2701](https://github.com/Rutu2701)

</div>

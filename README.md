# Diff Drive Plotter

## Project Overview

Diff Drive Plotter is a differential drive robot plotter system that converts images into vector paths for plotting. It features advanced image processing capabilities and offers both a Python GUI and a Web GUI for control and visualization.

## Key Features

*   **Image-to-Vector Conversion:**
    *   Robust image processing pipeline supporting various methods: CLAHE, adaptive thresholding, edge detection (Canny, Sobel, etc.), morphology, and skeletonization.
    *   Multiple vectorization strategies including contour extraction, distance transform centerlines, and Potrace integration for clean, SVG-like output.
    *   Adjustable parameters for fine-tuning image processing and vectorization for different types of input images.
*   **Path Planning & Kinematics:**
    *   Calculates precise robot movements based on configurable kinematics (wheel diameter, axle width, microsteps).
    *   Optimizes paths by reordering strokes and resampling points to minimize pen travel.
    *   Supports physical mirroring of paths.
*   **Dual Interface Control:**
    *   **Python GUI:** A Tkinter-based desktop application (`v17.py`) for image loading, parameter tuning, path preview, and direct plotting control.
    *   **Web GUI:** A modern, browser-based interface (`plotter_lab_v19.html`) offering a similar feature set with a user-friendly, multi-pane layout and real-time feedback.
*   **Arduino Firmware:**
    *   Low-level control for stepper motors and pen servo.
    *   Implements a robust serial communication protocol for receiving commands from the GUIs (steps, pen control, speed settings, servo configuration).
    *   Includes features like motor fault detection and emergency stop.
*   **Hardware Agnostic (within limits):** Designed to work with common stepper motor drivers and servos, configurable via serial commands.

## Technologies

*   **Python:** OpenCV (`opencv-python`), NumPy, Pillow, PySerial, `potrace` (optional).
*   **JavaScript:** Vanilla JS for Web GUI interactivity.
*   **Arduino/C++:** Standard Arduino libraries (`Servo.h`), direct hardware control.

## Getting Started

1.  **Clone the repository.**
2.  **Python GUI Setup:**
    *   Install dependencies: `pip install -r requirements.txt`
    *   Run the GUI: `python v17.py`
3.  **Web GUI Setup:**
    *   Serve the HTML file: `python -m http.server 8000` (or open `plotter_lab_v19.html` directly in a compatible browser).
4.  **Arduino Firmware:**
    *   Compile using Arduino CLI: `arduino-cli compile --fqbn arduino:avr:uno firmware.ino`
    *   Upload the compiled firmware to your Arduino board.
5.  **Connect:** Connect the Arduino to your computer via USB and select the correct serial port in the GUI.

## Serial Protocol

The firmware listens for the following serial commands:

*   `STEPS L:<n> R:<n>`: Execute steps for left and right motors.
*   `PEN DOWN` / `PEN UP`: Control the pen servo.
*   `SPEED <us>`: Set drawing speed.
*   `TSPEED <us>`: Set travel speed.
*   `SERVO <pin> <down> <up>`: Configure servo pin and angles.
*   `SERVO_ENABLE` / `SERVO_DISABLE`: Enable/disable servo action.
*   `PING`: Check connection status.
*   `STOP`: Emergency stop.

## Code Style

*   **Python:** 2-space indentation, `snake_case`, docstrings.
*   **Arduino/C++:** 2-space indentation, `camelCase`, `UPPER_SNAKE_CASE` for macros.
*   **HTML/CSS:** 2-space indentation, CSS variables for theming.

## Hardware Notes

*   **Motor Configuration:** 200 steps/rev with 32 microsteps. Default wheel diameter: 60mm, axle length: 120mm.
*   **Servo Configuration:** Default pin 13, pen down 30°, pen up 90°.
*   **Serial Settings:** Baud rate 115200, command timeout 2s, inter-command delay 2ms.

## Troubleshooting

*   **Python GUI:** Missing dependencies (`pip install -r requirements.txt`), import errors (Python 3.7+ required), serial port not found (check permissions/connection).
*   **Arduino:** Motors not moving (check enable pin/power), serial not responding (verify baud rate/USB), servo not moving (check wiring/voltage).

## Contributing

Refer to `AGENTS.md` for detailed build, test, and contribution guidelines. Follow established code style conventions. Commit messages should be descriptive and follow the imperative mood (e.g., "Add feature X").

---
*Last updated: May 2026*

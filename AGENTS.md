# AGENTS.md - Diff Drive Plotter

This document contains build/test commands and code style guidelines for the Diff Drive Plotter project.

## Project Overview

Diff Drive Plotter is a differential drive robot plotter system with:
- Python GUI application for image processing and path planning (`v18_arc_aware.py`)
- Arduino firmware for motor control and servo handling (`v6_arc_aware.ino`)
- Multi-algorithm midline detection with arc-aware speed modulation

## Build & Test Commands

### Python Environment Setup
```bash
# Install Python dependencies
pip install opencv-python Pillow pyserial numpy

# Optional: scikit-image for additional algorithms
pip install scikit-image
```

### Running the Application
```bash
# Run the Python plotter GUI
python v18_arc_aware.py
```

### Testing Arduino Code
```bash
# Compile and upload using Arduino IDE or PlatformIO
# No automated tests - manual verification via serial commands
```

### Development Commands
```bash
# Run with python debug output
python -m pdb v18_arc_aware.py

# Check Python syntax
python -m py_compile v18_arc_aware.py

# Check Arduino syntax (requires Arduino CLI)
arduino-cli compile --fqbn arduino:avr:uno v6_arc_aware.ino
```

## Code Style Guidelines

### Python Style
- **Formatting**: Use 2-space indentation (existing code uses this)
- **Imports**: Group imports by: standard libs, third-party, local modules
- **Naming**: 
  - snake_case for functions/variables
  - PascalCase for classes
  - UPPER_SNAKE_CASE for constants
- **Error Handling**: Use try/except blocks with specific exceptions
- **Documentation**: Use docstrings for public functions/classes
- **Types**: Type hints encouraged but not required

Example from existing code:
```python
def update_geometry(wheel_mm, axle_mm):
    """Update robot kinematics."""
    global WHEEL_MM, AXLE_MM, STEPS_PER_MM, STEPS_PER_DEG
    WHEEL_MM      = wheel_mm
    AXLE_MM       = axle_mm
    STEPS_PER_MM  = (FULL_STEPS * MICROSTEPS) / (math.pi * WHEEL_MM)
    STEPS_PER_DEG = (math.pi * AXLE_MM / 360.0) * STEPS_PER_MM
```

### Arduino/C++ Style
- **Formatting**: Use 2-space indentation (matching existing code)
- **Naming**:
  - camelCase for functions/variables
  - UPPER_SNAKE_CASE for macros/constants
- **Memory**: Be mindful of stack usage; avoid large local arrays
- **Serial Communication**: Always include timeout handling
- **Error Handling**: Check fault pins and report via serial

Example from existing code:
```cpp
void executeSteps(long stL, long stR) {
  if (stL == 0 && stR == 0) { Serial.println(F("OK")); return; }
  if (checkFault()) return;
  enableMotors();
  // ...
}
```

### File Organization
- **Python GUI**: Main application logic in `v18_arc_aware.py`
- **Arduino Firmware**: Motor control in `v6_arc_aware.ino`
- **Documentation**: Integration guide in `INTEGRATION_GUIDE_v18.md`

### Project-Specific Conventions

#### 1. Constants
- Define constants at module/file level
- Use descriptive names with units (e.g., `DEF_WHEEL_MM`, `BAUD`)
- Group related constants together

#### 2. Color Scheme
- Follow existing warm light theme colors in GUI
- Define colors as hex strings at top of GUI section

#### 3. Serial Protocol
- Commands follow format: `COMMAND param1:value param2:value`
- Always respond with `OK` or `ERR` messages
- Include timeout handling for all serial operations

#### 4. Kinematics Functions
- Keep pure math functions separate from GUI logic
- Document formulas and assumptions
- Use float for precision, int for steps

#### 5. Image Processing Pipeline
- Follow the established pipeline: CLAHE → Blur → Otsu → Midline Extraction
- Each step should be modular and configurable via UI

### Error Handling Guidelines

#### Python Errors
- Use messagebox for user-facing errors
- Log errors to both console and GUI log panel
- Include context in error messages (e.g., file names, parameter values)

#### Arduino Errors
- Check motor driver fault pins before moves
- Report hardware errors via serial immediately
- Implement safe shutdown on critical errors

### Testing Guidelines

#### Manual Testing Procedures
1. **Image Processing**: Test with sample images, verify midline extraction
2. **Path Planning**: Generate moves, verify step counts are reasonable
3. **Serial Communication**: Test all commands via serial monitor
4. **Motor Control**: Verify wheel directions and step accuracy
5. **Arc Detection**: Test with curved paths, verify speed modulation

#### Test Images
- Use simple line drawings for basic functionality
- Use complex drawings for algorithm validation
- Test with various stroke widths and curvatures

### Development Workflow

1. **Python Changes**:
   - Test GUI functionality locally
   - Verify image processing pipeline
   - Check log output for errors

2. **Arduino Changes**:
   - Compile and upload to hardware
   - Test serial commands individually
   - Verify motor movements match expected steps

3. **Integration Testing**:
   - Run full pipeline: image → moves → hardware
   - Monitor serial communication for timeouts/errors
   - Verify plot quality matches expectations

### Performance Considerations

#### Python
- Use NumPy operations for image processing
- Avoid blocking UI thread for long operations
- Use threading for serial communication and image processing

#### Arduino
- Minimize delay() calls in critical paths
- Use microsecond timing for motor control
- Keep serial command processing fast

### Documentation Requirements

#### Function Documentation
- Include docstrings for all public functions
- Document parameters, return values, and side effects
- Include mathematical formulas when applicable

#### Configuration
- Document all UI controls and their effects
- Include example values and recommended ranges
- Document hardware wiring and setup

### Version Control

#### Commit Messages
- Use present tense imperative: "Add feature" not "Added feature"
- Include context: "Fix arc speed calculation in tight curves"
- Reference hardware changes when applicable

#### Branching Strategy
- `main`: Stable releases
- `develop`: Integration branch
- Feature branches: `feature/arc-detection`, `bugfix/serial-timeout`

### Hardware Notes

#### Motor Configuration
- 200 steps/rev with 32 microsteps
- Default wheel diameter: 60mm
- Default axle length: 120mm

#### Servo Configuration
- Default pin: 13 (configurable)
- Pen down angle: 30°
- Pen up angle: 90°

#### Serial Settings
- Baud rate: 115200
- Command timeout: 2 seconds
- Inter-command delay: 2ms

### Troubleshooting Common Issues

#### Python GUI
- Missing dependencies: Install with `pip install -r requirements.txt`
- Import errors: Check Python version (3.7+ required)
- Serial port not found: Check permissions and cable connection

#### Arduino
- Motors not moving: Check enable pin and power supply
- Serial not responding: Verify baud rate and USB connection
- Servo not moving: Check wiring and voltage

### Future Development Areas

1. **Additional Midline Algorithms**: More detection methods
2. **Real-time Feedback**: Encoder integration for closed-loop control
3. **Web Interface**: Remote control and monitoring
4. **Advanced Path Optimization**: Minimize pen lifts and travel time
5. **Multi-material Support**: Different pen types and pressures

---
*Last updated: April 5, 2026*
*For project details, see INTEGRATION_GUIDE_v18.md*
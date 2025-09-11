# Thermal-Based Search-and-Rescue Rover

## Project Overview

This repository contains the software components for a Thermal-Based Search-and-Rescue Rover system. The software is designed to enable remote operation of a rover equipped with thermal imaging capabilities for search and rescue operations.

## Code Structure

### GUI Module (`software/gui/`)
- **main.py**: Entry point for GUI application, handles operator controls and displays thermal camera feed
- **controls.py**: Processes user inputs (WASD, joystick controls)
- **video_display.py**: Displays incoming thermal video stream from rover

### Rover Module (`software/rover/`)
- **main.py**: Entry point for rover control system
- **motor_control.py**: Controls rover motors and movement logic
- **thermal_camera.py**: Interfaces with thermal camera to capture frames
- **sensors.py**: Handles other rover-side sensors (GPS, IMU, etc.)
- **power_management.py**: Monitors battery level and reports status

### Wireless Module (`software/wireless/`)
- **comm.py**: Defines protocol for transmitting rover telemetry and receiving commands
- **receiver.py**: Runs on GUI side, receives rover data
- **sender.py**: Runs on rover side, sends telemetry and video stream

### Tests (`tests/`)
- **test_gui.py**: Placeholder for GUI module tests
- **test_rover.py**: Placeholder for rover module tests
- **test_wireless.py**: Placeholder for wireless module tests

## Setup Instructions

### Prerequisites
- Python 3.8+
- Required dependencies (see requirements.txt)

### Installation
1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Configure wireless communication settings

### Running the System
1. **GUI Side**: Run `python software/gui/main.py` to start the operator interface
2. **Rover Side**: Run `python software/rover/main.py` to start the rover control system
3. Establish wireless connection between GUI and rover

## Usage Instructions

### Connecting GUI to Rover
1. Start the rover system first
2. Launch the GUI application
3. Configure connection settings in the GUI
4. Establish wireless link

### Thermal Feed Display
- Thermal video stream is automatically displayed in the GUI
- Controls allow for zoom, contrast adjustment, and recording
- Real-time temperature readings are overlaid on the display

### Rover Control
- Use WASD keys or joystick for movement control
- Monitor battery status and sensor readings
- Adjust camera settings remotely

## Future Software Extensions

### Autonomy Features
- Path planning and obstacle avoidance
- Autonomous search patterns
- Return-to-base functionality

### GPS Integration
- Waypoint navigation
- Location tracking and mapping
- Search area coverage optimization

### Multiple Video Streams
- Support for additional cameras
- Picture-in-picture display modes
- Multi-camera recording capabilities

### Advanced Features
- Machine learning for thermal object detection
- Automated alert systems
- Data logging and analysis tools


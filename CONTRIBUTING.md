# Contributing to Thermal-Based Search-and-Rescue Rover

## Getting Started

1. **Clone the repository:**
   ```bash
   git clone https://github.com/CollierBlair/Capstone.git
   cd Capstone
   ```

2. **Set up your development environment:**
   ```bash
   # Create a virtual environment (recommended)
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   
   # Install dependencies (when requirements.txt is created)
   pip install -r requirements.txt
   ```

## Development Workflow

### Branch Strategy
- **main**: Production-ready code
- **feature/[module-name]**: New features (e.g., `feature/gui-controls`)
- **bugfix/[issue-number]**: Bug fixes

### Making Changes

1. **Create a new branch:**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** and test them

3. **Commit your changes:**
   ```bash
   git add .
   git commit -m "Add: Brief description of your changes"
   ```

4. **Push your branch:**
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Create a Pull Request** on GitHub to merge into main

### Code Standards

- **Python**: Follow PEP 8 style guidelines
- **Comments**: Add docstrings to all functions and classes
- **Testing**: Write tests for new functionality
- **Commits**: Use clear, descriptive commit messages

## Module Assignments

### GUI Module (`software/gui/`)
- **main.py**: Entry point and GUI coordination
- **controls.py**: User input handling (WASD, joystick)
- **video_display.py**: Thermal video stream display

### Rover Module (`software/rover/`)
- **main.py**: Rover control system coordination
- **motor_control.py**: Motor control and movement logic
- **thermal_camera.py**: Thermal camera interface
- **sensors.py**: Additional sensor handling
- **power_management.py**: Battery monitoring

### Wireless Module (`software/wireless/`)
- **comm.py**: Communication protocol definition
- **receiver.py**: GUI-side data reception
- **sender.py**: Rover-side data transmission

## Communication

- Use GitHub Issues for bug reports and feature requests
- Use Pull Request comments for code reviews
- Coordinate on which modules each person will work on

## Getting Help

- Check the README.md for project overview
- Look at existing code comments for implementation guidance
- Ask questions in GitHub Issues or team chat

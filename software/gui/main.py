"""
Entry point for GUI. Handles operator controls, displays thermal camera feed.

This module serves as the main interface for the search-and-rescue rover operator.
It coordinates between the control interface, video display, and wireless communication
to provide a comprehensive control system for the rover.

Key responsibilities:
- Initialize the GUI application
- Set up operator control interface
- Display thermal camera feed from rover
- Handle user input and command transmission
- Monitor rover status and telemetry
"""

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QKeySequence


class RoverControlGUI(QMainWindow):
    """
    Main GUI application for rover control.
    
    This class creates the main window and handles all user interactions
    for controlling the search-and-rescue rover.
    """
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thermal Rover Control v1.0")
        self.setGeometry(100, 100, 800, 600)
        
        # Initialize the UI
        self.init_ui()
        
        # Set up key tracking
        self.pressed_keys = set()
        
        # Timer for continuous key handling
        self.key_timer = QTimer()
        self.key_timer.timeout.connect(self.handle_continuous_keys)
        self.key_timer.start(50)  # 20 FPS for smooth movement
        
    def init_ui(self):
        """Initialize the user interface."""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Title
        title_label = QLabel("🔥 Thermal Rover Control")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Status display
        self.status_label = QLabel("Status: Ready - Use Arrow Keys to Control")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 10px; border-radius: 5px; }")
        main_layout.addWidget(self.status_label)
        
        # Control instructions
        instructions = QLabel("Controls:\n↑ Forward  ↓ Backward  ← Left  → Right")
        instructions.setFont(QFont("Arial", 10))
        instructions.setAlignment(Qt.AlignCenter)
        instructions.setStyleSheet("QLabel { color: #666; margin: 10px; }")
        main_layout.addWidget(instructions)
        
        # Movement display area
        self.movement_display = QLabel("Movement: None")
        self.movement_display.setFont(QFont("Arial", 14, QFont.Bold))
        self.movement_display.setAlignment(Qt.AlignCenter)
        self.movement_display.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 20px; border: 2px solid #ccc; border-radius: 10px; }")
        main_layout.addWidget(self.movement_display)
        
        # Control buttons (visual representation) - Keyboard layout
        # Create arrow buttons for visual reference
        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")
        
        # Style the buttons
        button_style = """
            QPushButton {
                font-size: 24px;
                font-weight: bold;
                padding: 20px;
                border: 2px solid #333;
                border-radius: 10px;
                background-color: #f0f0f0;
            }
            QPushButton:pressed {
                background-color: #4CAF50;
                color: white;
            }
        """
        
        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet(button_style)
            btn.setFixedSize(80, 80)
        
        # Create keyboard-style layout (inverted T)
        # Top row: Up arrow centered
        top_layout = QHBoxLayout()
        top_layout.addStretch()
        top_layout.addWidget(self.up_btn)
        top_layout.addStretch()
        
        # Bottom row: Left, Down, Right arrows
        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch()
        bottom_layout.addWidget(self.left_btn)
        bottom_layout.addWidget(self.down_btn)
        bottom_layout.addWidget(self.right_btn)
        bottom_layout.addStretch()
        
        # Combine top and bottom layouts
        button_layout = QVBoxLayout()
        button_layout.addLayout(top_layout)
        button_layout.addLayout(bottom_layout)
        
        main_layout.addLayout(button_layout)
        
        # Connection status
        self.connection_label = QLabel("Connection: Disconnected")
        self.connection_label.setFont(QFont("Arial", 10))
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet("QLabel { color: red; }")
        main_layout.addWidget(self.connection_label)
        
        # Add some spacing
        main_layout.addStretch()
        
    def keyPressEvent(self, event):
        """Handle key press events."""
        key = event.key()
        
        # Add key to pressed keys set
        self.pressed_keys.add(key)
        
        # Handle individual key presses
        if key == Qt.Key_Up:
            self.handle_movement("Forward")
            self.up_btn.setDown(True)
        elif key == Qt.Key_Down:
            self.handle_movement("Backward")
            self.down_btn.setDown(True)
        elif key == Qt.Key_Left:
            self.handle_movement("Left")
            self.left_btn.setDown(True)
        elif key == Qt.Key_Right:
            self.handle_movement("Right")
            self.right_btn.setDown(True)
        elif key == Qt.Key_Escape:
            self.close()
        
        # Don't call parent keyPressEvent to prevent default behavior
        event.accept()
    
    def keyReleaseEvent(self, event):
        """Handle key release events."""
        key = event.key()
        
        # Remove key from pressed keys set
        self.pressed_keys.discard(key)
        
        # Reset button states
        if key == Qt.Key_Up:
            self.up_btn.setDown(False)
        elif key == Qt.Key_Down:
            self.down_btn.setDown(False)
        elif key == Qt.Key_Left:
            self.left_btn.setDown(False)
        elif key == Qt.Key_Right:
            self.right_btn.setDown(False)
        
        # If no keys are pressed, stop movement
        if not self.pressed_keys:
            self.handle_movement("Stop")
        
        event.accept()
    
    def handle_continuous_keys(self):
        """Handle continuous key presses for smooth movement."""
        if not self.pressed_keys:
            return
        
        # Get the most recent key press for movement
        if Qt.Key_Up in self.pressed_keys:
            self.handle_movement("Forward")
        elif Qt.Key_Down in self.pressed_keys:
            self.handle_movement("Backward")
        elif Qt.Key_Left in self.pressed_keys:
            self.handle_movement("Left")
        elif Qt.Key_Right in self.pressed_keys:
            self.handle_movement("Right")
    
    def handle_movement(self, direction):
        """Handle movement commands."""
        self.movement_display.setText(f"Movement: {direction}")
        
        # Update status
        if direction == "Stop":
            self.status_label.setText("Status: Stopped")
            self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 10px; border-radius: 5px; }")
        else:
            self.status_label.setText(f"Status: Moving {direction}")
            self.status_label.setStyleSheet("QLabel { background-color: #4CAF50; color: white; padding: 10px; border-radius: 5px; }")
        
        # Here you would send the command to the rover
        # For now, just print to console
        print(f"Rover Command: {direction}")
        
        # TODO: Send command to rover via wireless communication
        # self.send_rover_command(direction)
    
    def send_rover_command(self, command):
        """Send command to rover (placeholder for future implementation)."""
        # This will be implemented when we integrate with wireless communication
        pass


def main():
    """Main function to run the GUI application."""
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = RoverControlGUI()
    window.show()
    
    # Start the application
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

# Note for Tyler, Probably going to use PyQt5 for the GUI. 
# I'm not sure how I will implement it but it should not be too hard.
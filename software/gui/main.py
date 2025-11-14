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
import json
import time
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QPushButton, QComboBox, 
                            QGroupBox, QGridLayout, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QKeySequence


class LoRaCommunicator(QThread):
    """
    Handles LoRa communication via serial port.
    
    This class manages the serial communication with the LoRa module,
    sending commands to the rover and receiving telemetry data.
    """
    
    # Signals for GUI updates
    connection_status_changed = pyqtSignal(bool)
    telemetry_received = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_connection = None
        self.is_connected = False
        self.port_name = None
        self.baud_rate = 9600  # Default for SX1262 LoRa modules
        
    def connect_to_lora(self, port_name: str, baud_rate: int = 9600):
        """Connect to LoRa module via serial port."""
        try:
            self.serial_connection = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.port_name = port_name
            self.baud_rate = baud_rate
            self.is_connected = True
            self.connection_status_changed.emit(True)
            print(f"Connected to LoRa module on {port_name} at {baud_rate} baud")
            return True
            
        except Exception as e:
            self.is_connected = False
            self.connection_status_changed.emit(False)
            self.error_occurred.emit(f"Failed to connect to LoRa: {str(e)}")
            print(f"LoRa connection error: {e}")
            return False
    
    def disconnect_from_lora(self):
        """Disconnect from LoRa module."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.is_connected = False
        self.connection_status_changed.emit(False)
        print("Disconnected from LoRa module")
    
    def send_command(self, command: str):
        """Send command to rover via LoRa."""
        if not self.is_connected or not self.serial_connection:
            print("Not connected to LoRa module")
            return False
        
        try:
            # Format command as JSON
            command_data = {
                "type": "movement",
                "command": command,
                "timestamp": time.time()
            }
            
            # Send command
            message = json.dumps(command_data) + "\n"
            self.serial_connection.write(message.encode())
            print(f"Sent command: {command}")
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"Failed to send command: {str(e)}")
            print(f"Command send error: {e}")
            return False
    
    def run(self):
        """Main thread loop for receiving telemetry data."""
        while self.is_connected and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    # Read telemetry data from rover
                    data = self.serial_connection.readline().decode().strip()
                    if data:
                        try:
                            telemetry = json.loads(data)
                            self.telemetry_received.emit(telemetry)
                        except json.JSONDecodeError:
                            print(f"Invalid JSON received: {data}")
                
                # Small delay to prevent excessive CPU usage
                self.msleep(10)
                
            except Exception as e:
                self.error_occurred.emit(f"Telemetry receive error: {str(e)}")
                print(f"Telemetry error: {e}")
                break


class RoverControlGUI(QMainWindow):
    """
    Main GUI application for rover control.
    
    This class creates the main window and handles all user interactions
    for controlling the search-and-rescue rover.
    """
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thermal Rover Control v1.0 - LoRa Ready")
        self.setGeometry(100, 100, 900, 700)
        
        # Initialize LoRa communication
        self.lora_comm = LoRaCommunicator()
        self.lora_comm.connection_status_changed.connect(self.update_connection_status)
        self.lora_comm.telemetry_received.connect(self.handle_telemetry)
        self.lora_comm.error_occurred.connect(self.handle_error)
        
        # Initialize the UI
        self.init_ui()
        
        # Set up key tracking
        self.pressed_keys = set()
        
        # Timer for continuous key handling
        self.key_timer = QTimer()
        self.key_timer.timeout.connect(self.handle_continuous_keys)
        self.key_timer.start(50)  # 20 FPS for smooth movement
        
        # Scan for available serial ports
        self.scan_serial_ports()
        
    def init_ui(self):
        """Initialize the user interface."""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout - horizontal split
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left side - Video feed area (takes up most space)
        video_layout = QVBoxLayout()
        
        # Video feed placeholder
        self.video_label = QLabel("Video Feed\n(Placeholder for thermal camera)")
        self.video_label.setFont(QFont("Arial", 14))
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("""
            QLabel { 
                background-color: #000; 
                color: #fff; 
                padding: 50px; 
                border: 2px solid #333; 
                border-radius: 10px;
                min-height: 400px;
            }
        """)
        video_layout.addWidget(self.video_label)
        
        # Status display for video area
        self.status_label = QLabel("Status: Ready - Use Arrow Keys to Control")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 10px; border-radius: 5px; }")
        video_layout.addWidget(self.status_label)
        
        # Add video layout to main layout (takes up most space)
        main_layout.addLayout(video_layout, 3)  # 3/4 of the space
        
        # Right side - Controls panel (bottom right area)
        controls_layout = QVBoxLayout()
        
        # Title
        title_label = QLabel("Thermal Rover Control")
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        controls_layout.addWidget(title_label)
        
        # Movement display area
        self.movement_display = QLabel("Movement: None")
        self.movement_display.setFont(QFont("Arial", 12, QFont.Bold))
        self.movement_display.setAlignment(Qt.AlignCenter)
        self.movement_display.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 15px; border: 2px solid #ccc; border-radius: 8px; }")
        controls_layout.addWidget(self.movement_display)
        
        # Control instructions
        instructions = QLabel("Controls:\n↑ Forward  ↓ Backward  ← Left  → Right")
        instructions.setFont(QFont("Arial", 9))
        instructions.setAlignment(Qt.AlignCenter)
        instructions.setStyleSheet("QLabel { color: #666; margin: 5px; }")
        controls_layout.addWidget(instructions)
        
        # Control buttons (visual representation) - Keyboard layout
        # Create arrow buttons for visual reference
        self.up_btn = QPushButton("↑")
        self.down_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")
        
        # Style the buttons
        button_style = """
            QPushButton {
                font-size: 20px;
                font-weight: bold;
                padding: 15px;
                border: 2px solid #333;
                border-radius: 8px;
                background-color: #f0f0f0;
            }
            QPushButton:pressed {
                background-color: #4CAF50;
                color: white;
            }
        """
        
        for btn in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            btn.setStyleSheet(button_style)
            btn.setFixedSize(60, 60)
        
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
        
        controls_layout.addLayout(button_layout)
        
        # LoRa Connection Settings
        lora_group = QGroupBox("LoRa Settings")
        lora_layout = QVBoxLayout()
        
        # Serial port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        port_layout.addWidget(self.port_combo)
        lora_layout.addLayout(port_layout)
        
        # Baud rate selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("9600")
        baud_layout.addWidget(self.baud_combo)
        lora_layout.addLayout(baud_layout)
        
        # Connect/Disconnect buttons
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_to_lora)
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 6px; }")
        lora_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_from_lora)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }")
        lora_layout.addWidget(self.disconnect_btn)
        
        lora_group.setLayout(lora_layout)
        controls_layout.addWidget(lora_group)
        
        # Connection status
        self.connection_label = QLabel("LoRa: Disconnected")
        self.connection_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
        controls_layout.addWidget(self.connection_label)
        
        # Telemetry display
        self.telemetry_label = QLabel("Telemetry: No data")
        self.telemetry_label.setFont(QFont("Arial", 9))
        self.telemetry_label.setAlignment(Qt.AlignCenter)
        self.telemetry_label.setStyleSheet("QLabel { background-color: #f5f5f5; padding: 6px; border-radius: 3px; }")
        controls_layout.addWidget(self.telemetry_label)
        
        # Add controls to main layout (1/4 of the space)
        main_layout.addLayout(controls_layout, 1)
        
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
        
        # Send command to rover via LoRa
        print(f"Rover Command: {direction}")
        self.send_rover_command(direction)
    
    def send_rover_command(self, command):
        """Send command to rover via LoRa communication."""
        if self.lora_comm.is_connected:
            success = self.lora_comm.send_command(command)
            if not success:
                print(f"Failed to send command: {command}")
        else:
            print(f"LoRa not connected. Command: {command}")
    
    def scan_serial_ports(self):
        """Scan for available serial ports and populate the combo box."""
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        
        for port in ports:
            # On Mac, look for USB serial devices (common for LoRa modules)
            if "usb" in port.device.lower() or "tty.usb" in port.device.lower():
                self.port_combo.addItem(f"{port.device} - {port.description}")
            else:
                self.port_combo.addItem(f"{port.device} - {port.description}")
        
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No serial ports found")
            self.connect_btn.setEnabled(False)
    
    def connect_to_lora(self):
        """Connect to LoRa module using selected port and baud rate."""
        if self.port_combo.currentText() == "No serial ports found":
            QMessageBox.warning(self, "No Ports", "No serial ports available!")
            return
        
        # Extract port name from combo box text
        port_text = self.port_combo.currentText()
        port_name = port_text.split(" - ")[0]
        baud_rate = int(self.baud_combo.currentText())
        
        # Attempt connection
        if self.lora_comm.connect_to_lora(port_name, baud_rate):
            self.lora_comm.start()  # Start the telemetry receiving thread
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
        else:
            QMessageBox.critical(self, "Connection Failed", 
                               f"Failed to connect to LoRa module on {port_name}")
    
    def disconnect_from_lora(self):
        """Disconnect from LoRa module."""
        self.lora_comm.disconnect_from_lora()
        self.lora_comm.quit()
        self.lora_comm.wait()
        
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
    
    def update_connection_status(self, connected):
        """Update connection status display."""
        if connected:
            self.connection_label.setText("LoRa Status: Connected")
            self.connection_label.setStyleSheet("QLabel { color: green; background-color: #e8f5e8; padding: 10px; border-radius: 5px; }")
        else:
            self.connection_label.setText("LoRa Status: Disconnected")
            self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 10px; border-radius: 5px; }")
    
    def handle_telemetry(self, telemetry_data):
        """Handle telemetry data received from rover."""
        # Update telemetry display
        telemetry_text = f"Battery: {telemetry_data.get('battery', 'N/A')}% | "
        telemetry_text += f"GPS: {telemetry_data.get('gps', 'N/A')} | "
        telemetry_text += f"Status: {telemetry_data.get('status', 'N/A')}"
        
        self.telemetry_label.setText(f"Telemetry: {telemetry_text}")
        print(f"Received telemetry: {telemetry_data}")
    
    def handle_error(self, error_message):
        """Handle communication errors."""
        QMessageBox.warning(self, "Communication Error", error_message)
        print(f"LoRa Error: {error_message}")
    
    def closeEvent(self, event):
        """Handle application close event."""
        if self.lora_comm.is_connected:
            self.disconnect_from_lora()
        event.accept()


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
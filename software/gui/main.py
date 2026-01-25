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
import urllib.request
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QPushButton, QGroupBox, 
                            QMessageBox)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap


class VideoReceiver(QThread):
    """
    Handles video stream reception from Pi via WiFi.
    
    This class manages the HTTP connection to the Pi's video stream,
    decodes JPEG frames, and emits them for display in the GUI.
    """
    
    # Signals for GUI updates
    frame_ready = pyqtSignal(QPixmap)
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, video_url="http://192.168.1.200:5000/video"):
        super().__init__()
        self.video_url = video_url
        self.is_connected = False
        self.stream = None
        self.running = False
        
    def connect_to_stream(self):
        """Connect to video stream from Pi."""
        try:
            self.stream = urllib.request.urlopen(self.video_url, timeout=5)
            self.is_connected = True
            self.running = True
            self.connection_status_changed.emit(True)
            print(f"Connected to video stream at {self.video_url}")
            return True
        except Exception as e:
            self.is_connected = False
            self.connection_status_changed.emit(False)
            self.error_occurred.emit(f"Failed to connect to video stream: {str(e)}")
            print(f"Video stream connection error: {e}")
            return False
    
    def disconnect_from_stream(self):
        """Disconnect from video stream."""
        self.running = False
        if self.stream:
            try:
                self.stream.close()
            except:
                pass
        self.stream = None
        self.is_connected = False
        self.connection_status_changed.emit(False)
        print("Disconnected from video stream")
    
    def run(self):
        """Main thread loop for receiving and decoding video frames."""
        bytes_data = b''
        
        while self.running and self.is_connected:
            try:
                if not self.stream:
                    self.msleep(100)
                    continue
                
                # Read a chunk of the stream
                chunk = self.stream.read(1024)
                if not chunk:
                    # Stream ended, try to reconnect
                    self.error_occurred.emit("Video stream ended, attempting to reconnect...")
                    if self.connect_to_stream():
                        bytes_data = b''
                        continue
                    else:
                        break
                
                bytes_data += chunk
                
                # Look for JPEG start and end markers
                start_marker = bytes_data.find(b'\xff\xd8')
                end_marker = bytes_data.find(b'\xff\xd9')
                
                if start_marker != -1 and end_marker != -1 and end_marker > start_marker:
                    # Extract JPEG frame
                    jpg = bytes_data[start_marker:end_marker + 2]
                    bytes_data = bytes_data[end_marker + 2:]
                    
                    # Decode JPEG directly into QImage
                    qimage = QImage()
                    if qimage.loadFromData(jpg):
                        # Convert to QPixmap and emit signal
                        pixmap = QPixmap.fromImage(qimage)
                        # Scale to fit video label (optional - can be done in GUI)
                        self.frame_ready.emit(pixmap)
                
                # Small delay to prevent excessive CPU usage
                self.msleep(10)
                
            except Exception as e:
                self.error_occurred.emit(f"Video receive error: {str(e)}")
                print(f"Video error: {e}")
                # Try to reconnect after error
                self.msleep(1000)
                if self.running:
                    self.connect_to_stream()
                    bytes_data = b''


class RoverControlGUI(QMainWindow):
    """
    Main GUI application for rover control.
    
    This class creates the main window and handles all user interactions
    for controlling the search-and-rescue rover.
    """
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thermal Rover Control v1.0")
        self.setGeometry(100, 100, 900, 700)
        
        # Initialize video receiver
        self.video_receiver = VideoReceiver()
        self.video_receiver.frame_ready.connect(self.update_video_frame)
        self.video_receiver.connection_status_changed.connect(self.update_connection_status)
        self.video_receiver.error_occurred.connect(self.handle_error)
        
        # Initialize the UI
        self.init_ui()
        
        # Set up key tracking
        self.pressed_keys = set()
        
        # Timer for continuous key handling
        self.key_timer = QTimer()
        self.key_timer.timeout.connect(self.handle_continuous_keys)
        self.key_timer.start(50)  # 20 FPS for smooth movement
        
        # Auto-connect to video stream on startup
        self.connect_to_video()
        
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
        
        # Video feed display
        self.video_label = QLabel("Connecting to video stream...")
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
        self.video_label.setScaledContents(True)  # Scale video to fit label
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
        
        # Video Connection Settings
        video_group = QGroupBox("Video Stream")
        video_layout = QVBoxLayout()
        
        # Connect/Disconnect buttons
        self.connect_btn = QPushButton("Connect to Video")
        self.connect_btn.clicked.connect(self.connect_to_video)
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 6px; }")
        video_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_from_video)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }")
        video_layout.addWidget(self.disconnect_btn)
        
        video_group.setLayout(video_layout)
        controls_layout.addWidget(video_group)
        
        # Connection status
        self.connection_label = QLabel("Video: Disconnected")
        self.connection_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
        controls_layout.addWidget(self.connection_label)
        
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
        
        # Send command to rover (placeholder for future command implementation)
        print(f"Rover Command: {direction}")
        # TODO: Implement command sending via wireless communication
    
    def connect_to_video(self):
        """Connect to video stream from Pi."""
        if self.video_receiver.connect_to_stream():
            self.video_receiver.start()  # Start the video receiving thread
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        else:
            QMessageBox.warning(self, "Connection Failed", 
                             "Failed to connect to video stream.\n\n"
                             "Make sure:\n"
                             "1. Pi is running sender.py\n"
                             "2. Both devices are on the same WiFi network\n"
                             "3. Pi IP is 192.168.1.200")
    
    def disconnect_from_video(self):
        """Disconnect from video stream."""
        self.video_receiver.disconnect_from_stream()
        self.video_receiver.quit()
        self.video_receiver.wait()
        
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.video_label.setText("Video Feed\n(Disconnected)")
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
    
    def update_video_frame(self, pixmap):
        """Update video label with new frame."""
        # Scale pixmap to fit label while maintaining aspect ratio
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(), 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled_pixmap)
    
    def update_connection_status(self, connected):
        """Update connection status display."""
        if connected:
            self.connection_label.setText("Video: Connected")
            self.connection_label.setStyleSheet("QLabel { color: green; background-color: #e8f5e8; padding: 10px; border-radius: 5px; }")
        else:
            self.connection_label.setText("Video: Disconnected")
            self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 10px; border-radius: 5px; }")
    
    def handle_error(self, error_message):
        """Handle communication errors."""
        print(f"Video Error: {error_message}")
        # Don't show message box for every error to avoid spam
        # Only show critical errors
        if "Failed to connect" in error_message:
            QMessageBox.warning(self, "Connection Error", error_message)
    
    def closeEvent(self, event):
        """Handle application close event."""
        if self.video_receiver.is_connected:
            self.disconnect_from_video()
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
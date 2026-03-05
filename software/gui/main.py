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
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QPushButton, QComboBox, QGroupBox, 
                            QMessageBox, QFrame, QDesktopWidget, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap


class STM32Communicator(QThread):
    """Serial communication with STM32 sender (4-byte key state protocol)."""
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial_connection = None
        self.is_connected = False

    def connect_to_sender(self, port_name: str, baud_rate: int):
        try:
            self.serial_connection = serial.Serial(port=port_name, baudrate=baud_rate, timeout=1)
            # Verify we can actually write (catches virtual/ghost ports that open but don't work)
            self.serial_connection.write(bytes([0x05, 0x05, 0x05, 0x05]))
            self.is_connected = True
            self.connection_status_changed.emit(True)
            return True
        except Exception as e:
            if self.serial_connection and self.serial_connection.is_open:
                try:
                    self.serial_connection.close()
                except Exception:
                    pass
                self.serial_connection = None
            self.is_connected = False
            self.connection_status_changed.emit(False)
            self.error_occurred.emit(str(e))
            return False

    def disconnect_from_sender(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.serial_connection = None
        self.is_connected = False
        self.connection_status_changed.emit(False)

    def send_key_state(self, up, left, down, right):
        if not self.is_connected or not self.serial_connection:
            return
        try:
            self.serial_connection.write(bytes([up, left, down, right]))
            self.serial_connection.flush()  # like test_ser.py - send immediately
        except Exception as e:
            self.error_occurred.emit(str(e))

    def run(self):
        while self.is_connected:
            self.msleep(10)


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
    
    def __init__(self, video_url="http://100.70.10.53:5000/video"):
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
        # Size window to fit available screen (works on any laptop)
        desktop = QDesktopWidget()
        available = desktop.availableGeometry()
        margin = 20
        self.setGeometry(
            available.x() + margin,
            available.y() + margin,
            available.width() - 2 * margin,
            available.height() - 2 * margin
        )
        
        # Initialize video receiver (port 5000)
        self.video_receiver = VideoReceiver(video_url="http://100.70.10.57:5000/video")
        self.video_receiver.frame_ready.connect(self.update_video_frame)
        self.video_receiver.connection_status_changed.connect(self.update_connection_status)
        self.video_receiver.error_occurred.connect(self.handle_error)

        # Initialize thermal receiver (port 5001)
        self.thermal_receiver = VideoReceiver(video_url="http://100.70.10.57:5001/thermal")
        self.thermal_receiver.frame_ready.connect(self.update_thermal_frame)
        self.thermal_receiver.connection_status_changed.connect(self.update_thermal_connection_status)
        self.thermal_receiver.error_occurred.connect(self.handle_thermal_error)

        # Initialize STM32 serial communicator
        self.stm32_comm = STM32Communicator()
        self.stm32_comm.connection_status_changed.connect(self.update_stm32_status)
        self.stm32_comm.error_occurred.connect(self.handle_stm32_error)
        
        # Initialize the UI
        self.init_ui()
        
        # Set up key tracking
        self.pressed_keys = set()
        
        # Timer for continuous key handling and STM32 transmission
        self.key_timer = QTimer()
        self.key_timer.timeout.connect(self.handle_continuous_keys)
        self.key_timer.start(50)  # 20 FPS for smooth movement
        
        # Scan serial ports for STM32
        self.scan_serial_ports()
        
        # Auto-connect to video stream on startup
        self.connect_to_video()
        
    def init_ui(self):
        """Initialize the user interface."""
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout - controls have fixed width so they're never pushed off screen
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left side - Video feeds stack: top = regular, bottom = thermal (both auto-fill)
        video_container = QWidget()
        video_layout = QVBoxLayout(video_container)
        video_layout.setContentsMargins(0, 0, 0, 0)
        video_layout.setSpacing(5)

        # Feeds side by side
        feeds_layout = QHBoxLayout()
        feeds_layout.setSpacing(5)

        # Regular video feed (port 5000) - left
        self.video_label = QLabel("Connecting...")
        self.video_label.setFont(QFont("Arial", 12))
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("""
            QLabel { background-color: #000; color: #fff; padding: 4px;
                     border: 2px solid #333; border-radius: 8px; }
        """)
        self.video_label.setScaledContents(True)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        feeds_layout.addWidget(self.video_label, 1)

        # Thermal video feed (port 5001) - right
        self.thermal_label = QLabel("Thermal: Off")
        self.thermal_label.setFont(QFont("Arial", 12))
        self.thermal_label.setAlignment(Qt.AlignCenter)
        self.thermal_label.setStyleSheet("""
            QLabel { background-color: #1a1a1a; color: #ff9800; padding: 4px;
                     border: 2px solid #333; border-radius: 8px; }
        """)
        self.thermal_label.setScaledContents(True)
        self.thermal_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        feeds_layout.addWidget(self.thermal_label, 1)

        video_layout.addLayout(feeds_layout, 1)
        
        # Status display for video area
        self.status_label = QLabel("Status: Ready - Use Arrow Keys to Control")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: #e0e0e0; padding: 10px; border-radius: 5px; }")
        video_layout.addWidget(self.status_label)

        # Right side - Controls panel (fixed width so always visible, even in fullscreen)
        controls_container = QFrame()
        controls_container.setFixedWidth(320)
        controls_layout = QVBoxLayout(controls_container)
        
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
        instructions = QLabel("Controls: Arrow keys or W A S D")
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
        video_group_layout = QVBoxLayout()
        
        # Regular video (port 5000) - Connect/Disconnect
        self.connect_btn = QPushButton("Connect to Video")
        self.connect_btn.clicked.connect(self.connect_to_video)
        self.connect_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 6px; }")
        video_group_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = QPushButton("Disconnect Video")
        self.disconnect_btn.clicked.connect(self.disconnect_from_video)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }")
        video_group_layout.addWidget(self.disconnect_btn)
        
        # Thermal feed (port 5001) - Connect/Disconnect
        self.thermal_connect_btn = QPushButton("Connect to Thermal")
        self.thermal_connect_btn.clicked.connect(self.connect_to_thermal)
        self.thermal_connect_btn.setStyleSheet("QPushButton { background-color: #ff9800; color: white; padding: 6px; }")
        video_group_layout.addWidget(self.thermal_connect_btn)
        
        self.thermal_disconnect_btn = QPushButton("Disconnect Thermal")
        self.thermal_disconnect_btn.clicked.connect(self.disconnect_from_thermal)
        self.thermal_disconnect_btn.setEnabled(False)
        self.thermal_disconnect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }")
        video_group_layout.addWidget(self.thermal_disconnect_btn)
        
        video_group.setLayout(video_group_layout)
        controls_layout.addWidget(video_group)
        
        # Connection status (video and thermal)
        self.connection_label = QLabel("Video: Disconnected")
        self.connection_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
        controls_layout.addWidget(self.connection_label)

        self.thermal_connection_label = QLabel("Thermal: Disconnected")
        self.thermal_connection_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.thermal_connection_label.setAlignment(Qt.AlignCenter)
        self.thermal_connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
        controls_layout.addWidget(self.thermal_connection_label)

        # STM32 Serial
        stm32_group = QGroupBox("STM32 Serial")
        stm32_layout = QVBoxLayout()
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(140)
        port_row.addWidget(self.port_combo)
        stm32_layout.addLayout(port_row)
        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")  # match test_ser.py and sender
        baud_row.addWidget(self.baud_combo)
        stm32_layout.addLayout(baud_row)
        self.stm32_connect_btn = QPushButton("Connect STM32")
        self.stm32_connect_btn.clicked.connect(self.connect_to_stm32)
        self.stm32_connect_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 6px; }")
        stm32_layout.addWidget(self.stm32_connect_btn)
        self.stm32_disconnect_btn = QPushButton("Disconnect STM32")
        self.stm32_disconnect_btn.clicked.connect(self.disconnect_from_stm32)
        self.stm32_disconnect_btn.setEnabled(False)
        self.stm32_disconnect_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 6px; }")
        stm32_layout.addWidget(self.stm32_disconnect_btn)
        stm32_group.setLayout(stm32_layout)
        controls_layout.addWidget(stm32_group)
        self.stm32_status_label = QLabel("STM32: Disconnected")
        self.stm32_status_label.setFont(QFont("Arial", 10, QFont.Bold))
        self.stm32_status_label.setAlignment(Qt.AlignCenter)
        self.stm32_status_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
        controls_layout.addWidget(self.stm32_status_label)

        # Add to main layout: video takes remaining space, controls stay fixed at 320px
        main_layout.addWidget(video_container, 1)
        main_layout.addWidget(controls_container, 0)
        
    def _up_pressed(self):
        return Qt.Key_Up in self.pressed_keys or Qt.Key_W in self.pressed_keys
    def _down_pressed(self):
        return Qt.Key_Down in self.pressed_keys or Qt.Key_S in self.pressed_keys
    def _left_pressed(self):
        return Qt.Key_Left in self.pressed_keys or Qt.Key_A in self.pressed_keys
    def _right_pressed(self):
        return Qt.Key_Right in self.pressed_keys or Qt.Key_D in self.pressed_keys

    def keyPressEvent(self, event):
        """Handle key press events (Arrow keys and WASD)."""
        key = event.key()
        self.pressed_keys.add(key)
        if key in (Qt.Key_Up, Qt.Key_W):
            self.handle_movement("Forward")
            self.up_btn.setDown(True)
        elif key in (Qt.Key_Down, Qt.Key_S):
            self.handle_movement("Backward")
            self.down_btn.setDown(True)
        elif key in (Qt.Key_Left, Qt.Key_A):
            self.handle_movement("Left")
            self.left_btn.setDown(True)
        elif key in (Qt.Key_Right, Qt.Key_D):
            self.handle_movement("Right")
            self.right_btn.setDown(True)
        elif key == Qt.Key_Escape:
            self.close()
        event.accept()

    def keyReleaseEvent(self, event):
        """Handle key release events."""
        key = event.key()
        self.pressed_keys.discard(key)
        if key in (Qt.Key_Up, Qt.Key_W):
            self.up_btn.setDown(False)
        elif key in (Qt.Key_Down, Qt.Key_S):
            self.down_btn.setDown(False)
        elif key in (Qt.Key_Left, Qt.Key_A):
            self.left_btn.setDown(False)
        elif key in (Qt.Key_Right, Qt.Key_D):
            self.right_btn.setDown(False)
        if not (self._up_pressed() or self._down_pressed() or self._left_pressed() or self._right_pressed()):
            self.handle_movement("Stop")
        event.accept()

    def handle_continuous_keys(self):
        """Handle continuous key presses and send STM32 4-byte key state."""
        if self._up_pressed():
            self.handle_movement("Forward")
        elif self._down_pressed():
            self.handle_movement("Backward")
        elif self._left_pressed():
            self.handle_movement("Left")
        elif self._right_pressed():
            self.handle_movement("Right")
        # STM32 protocol: 4 bytes [up, left, down, right], 0xA0 = pressed, 0x05 = released
        if self.stm32_comm.is_connected:
            up    = 0xA0 if self._up_pressed()    else 0x05
            left  = 0xA0 if self._left_pressed()  else 0x05
            down  = 0xA0 if self._down_pressed()  else 0x05
            right = 0xA0 if self._right_pressed() else 0x05
            self.stm32_comm.send_key_state(up, left, down, right)
    
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
        
        print(f"Rover Command: {direction}")

    # Ports that are virtual/system (not real USB serial like STM32). Don't list or allow connect.
    STM32_EXCLUDED_PORTS = (
        "wlan-debug", "debug-console", "Bluetooth-Incoming-Port", "Bluetooth-Modem",
        "debug", "Bluetooth-", "wlan-", "tty.debug-",
    )

    def _is_excluded_port(self, port_name: str) -> bool:
        return any(x in port_name for x in self.STM32_EXCLUDED_PORTS)

    def scan_serial_ports(self):
        """Populate port combo for STM32 serial (only real USB-type ports)."""
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            if self._is_excluded_port(p.device):
                continue
            self.port_combo.addItem(f"{p.device} - {p.description}")
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports found")

    def connect_to_stm32(self):
        """Connect to STM32 sender over serial."""
        if self.port_combo.currentText() == "No ports found":
            QMessageBox.warning(self, "No Ports", "No serial ports available.")
            return
        port = self.port_combo.currentText().split(" - ")[0]
        if self._is_excluded_port(port):
            self.update_stm32_status(False)
            QMessageBox.warning(self, "Can't connect to STM32",
                "This port is for system use, not the STM32.\n\n"
                "Plug in the STM32 via USB and choose its port (e.g. usbmodem or usbserial).")
            return
        baud = int(self.baud_combo.currentText())
        if self.stm32_comm.connect_to_sender(port, baud):
            self.stm32_comm.start()
            self.stm32_connect_btn.setEnabled(False)
            self.stm32_disconnect_btn.setEnabled(True)
        else:
            # connect_to_sender already emitted False and error_occurred (one dialog); keep UI disconnected
            self.update_stm32_status(False)

    def disconnect_from_stm32(self):
        """Disconnect from STM32 sender."""
        self.stm32_comm.disconnect_from_sender()
        self.stm32_comm.quit()
        self.stm32_comm.wait(500)
        self.stm32_connect_btn.setEnabled(True)
        self.stm32_disconnect_btn.setEnabled(False)

    def update_stm32_status(self, connected):
        """Update STM32 connection status label."""
        if connected:
            self.stm32_status_label.setText("STM32: Connected")
            self.stm32_status_label.setStyleSheet("QLabel { color: green; background-color: #e8f5e8; padding: 8px; border-radius: 5px; }")
        else:
            self.stm32_status_label.setText("STM32: Disconnected")
            self.stm32_status_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")

    def handle_stm32_error(self, msg):
        QMessageBox.warning(self, "Can't connect to STM32", msg)

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
                padding: 8px; 
                border: 2px solid #333; 
                border-radius: 10px;
            }
        """)

    def connect_to_thermal(self):
        """Connect to thermal stream from Pi (port 5001)."""
        if self.thermal_receiver.connect_to_stream():
            self.thermal_receiver.start()
            self.thermal_connect_btn.setEnabled(False)
            self.thermal_disconnect_btn.setEnabled(True)
        else:
            QMessageBox.warning(self, "Connection Failed",
                "Failed to connect to thermal stream.\n\n"
                "Make sure:\n"
                "1. Pi is running thermal_sender.py\n"
                "2. Both devices are on the same WiFi network\n"
                "3. Thermal server is at 100.70.10.57:5001")

    def disconnect_from_thermal(self):
        """Disconnect from thermal stream."""
        self.thermal_receiver.disconnect_from_stream()
        self.thermal_receiver.quit()
        self.thermal_receiver.wait()

        self.thermal_connect_btn.setEnabled(True)
        self.thermal_disconnect_btn.setEnabled(False)
        self.thermal_label.setText("Thermal: Disconnected")
        self.thermal_label.setStyleSheet("""
            QLabel { 
                background-color: #1a1a1a; 
                color: #ff9800; 
                padding: 8px; 
                border: 2px solid #333; 
                border-radius: 10px;
            }
        """)

    def handle_thermal_error(self, error_message):
        """Handle thermal stream errors."""
        print(f"Thermal Error: {error_message}")
        if "Failed to connect" in error_message:
            QMessageBox.warning(self, "Thermal Connection Error", error_message)
    
    def update_video_frame(self, pixmap):
        """Update video label with new frame."""
        self.video_label.setPixmap(pixmap)

    def update_thermal_frame(self, pixmap):
        """Update thermal label with new frame."""
        self.thermal_label.setPixmap(pixmap)
    
    def update_connection_status(self, connected):
        """Update connection status display."""
        if connected:
            self.connection_label.setText("Video: Connected")
            self.connection_label.setStyleSheet("QLabel { color: green; background-color: #e8f5e8; padding: 10px; border-radius: 5px; }")
        else:
            self.connection_label.setText("Video: Disconnected")
            self.connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 10px; border-radius: 5px; }")

    def update_thermal_connection_status(self, connected):
        """Update thermal connection status display."""
        if connected:
            self.thermal_connection_label.setText("Thermal: Connected")
            self.thermal_connection_label.setStyleSheet("QLabel { color: green; background-color: #e8f5e8; padding: 8px; border-radius: 5px; }")
        else:
            self.thermal_connection_label.setText("Thermal: Disconnected")
            self.thermal_connection_label.setStyleSheet("QLabel { color: red; background-color: #ffebee; padding: 8px; border-radius: 5px; }")
    
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
        if self.thermal_receiver.is_connected:
            self.disconnect_from_thermal()
        if self.stm32_comm.is_connected:
            self.disconnect_from_stm32()
        event.accept()


def main():
    """Main function to run the GUI application."""
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = RoverControlGUI()
    window.showMaximized()  # Fill screen on any laptop
    
    # Start the application
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

# Note for Tyler, Probably going to use PyQt5 for the GUI. 
# I'm not sure how I will implement it but it should not be too hard.
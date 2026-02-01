import sys
import time
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QComboBox,
                             QGroupBox, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont


# ================= STM32 COMM =================
class STM32Communicator(QThread):
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial_connection = None
        self.is_connected = False

    def connect_to_sender(self, port_name: str, baud_rate: int):
        try:
            self.serial_connection = serial.Serial(port=port_name,
                                                   baudrate=baud_rate,
                                                   timeout=1)
            self.is_connected = True
            self.connection_status_changed.emit(True)
            return True
        except Exception as e:
            self.connection_status_changed.emit(False)
            self.error_occurred.emit(str(e))
            return False

    def disconnect_from_sender(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.is_connected = False
        self.connection_status_changed.emit(False)

    # RAW 4-BYTE PROTOCOL
    def send_key_state(self, up, left, down, right):
        if not self.is_connected or not self.serial_connection:
            return
        try:
            packet = bytes([up, left, down, right])
            self.serial_connection.write(packet)
        except Exception as e:
            self.error_occurred.emit(str(e))

    def run(self):
        while self.is_connected:
            self.msleep(10)


# ================= GUI =================
class RoverControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Thermal Rover Control v1.0")
        self.setGeometry(100, 100, 900, 700)

        self.stm32_comm = STM32Communicator()
        self.stm32_comm.connection_status_changed.connect(self.update_connection_status)
        self.stm32_comm.error_occurred.connect(self.handle_error)

        self.pressed_keys = set()
        self.init_ui()
        self.scan_serial_ports()

        # 50ms transmission loop
        self.key_timer = QTimer()
        self.key_timer.timeout.connect(self.handle_continuous_keys)
        self.key_timer.start(50)

    # ================= UI =================
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        video_layout = QVBoxLayout()
        self.video_label = QLabel("Video Feed\n(Placeholder)")
        self.video_label.setFont(QFont("Arial", 14))
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background:black;color:white;padding:50px;border-radius:10px;")
        video_layout.addWidget(self.video_label)

        self.status_label = QLabel("Status: Ready")
        self.status_label.setAlignment(Qt.AlignCenter)
        video_layout.addWidget(self.status_label)

        main_layout.addLayout(video_layout, 3)

        controls_layout = QVBoxLayout()

        self.movement_display = QLabel("Movement: None")
        self.movement_display.setFont(QFont("Arial", 12, QFont.Bold))
        self.movement_display.setAlignment(Qt.AlignCenter)
        controls_layout.addWidget(self.movement_display)

        btn_style = """
        QPushButton {
            font-size: 20px;
            font-weight: bold;
            padding: 15px;
            border-radius: 10px;
            color: white;
        }
        """

        self.up_btn = QPushButton("↑")
        self.up_btn.setStyleSheet(btn_style + "background:#4CAF50;")

        self.down_btn = QPushButton("↓")
        self.down_btn.setStyleSheet(btn_style + "background:#f44336;")

        self.left_btn = QPushButton("←")
        self.left_btn.setStyleSheet(btn_style + "background:#2196F3;")

        self.right_btn = QPushButton("→")
        self.right_btn.setStyleSheet(btn_style + "background:#FF9800;")

        for b in [self.up_btn, self.down_btn, self.left_btn, self.right_btn]:
            b.setFixedSize(60, 60)

        top = QHBoxLayout()
        top.addStretch(); top.addWidget(self.up_btn); top.addStretch()

        bottom = QHBoxLayout()
        bottom.addStretch()
        bottom.addWidget(self.left_btn)
        bottom.addWidget(self.down_btn)
        bottom.addWidget(self.right_btn)
        bottom.addStretch()

        controls_layout.addLayout(top)
        controls_layout.addLayout(bottom)

        group = QGroupBox("STM32 Settings")
        group_layout = QVBoxLayout()

        self.port_combo = QComboBox()
        group_layout.addWidget(self.port_combo)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        group_layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_to_sender)
        group_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_from_sender)
        self.disconnect_btn.setEnabled(False)
        group_layout.addWidget(self.disconnect_btn)

        group.setLayout(group_layout)
        controls_layout.addWidget(group)

        self.connection_label = QLabel("STM32: Disconnected")
        controls_layout.addWidget(self.connection_label)

        main_layout.addLayout(controls_layout, 1)

    # ================= KEY EVENTS (WASD) =================
    def keyPressEvent(self, e):
        self.pressed_keys.add(e.key())

        if e.key() == Qt.Key_W: self.up_btn.setDown(True)
        if e.key() == Qt.Key_S: self.down_btn.setDown(True)
        if e.key() == Qt.Key_A: self.left_btn.setDown(True)
        if e.key() == Qt.Key_D: self.right_btn.setDown(True)

    def keyReleaseEvent(self, e):
        self.pressed_keys.discard(e.key())

        if e.key() == Qt.Key_W: self.up_btn.setDown(False)
        if e.key() == Qt.Key_S: self.down_btn.setDown(False)
        if e.key() == Qt.Key_A: self.left_btn.setDown(False)
        if e.key() == Qt.Key_D: self.right_btn.setDown(False)

    # TRANSMIT LOOP
    def handle_continuous_keys(self):
        up    = 0xA0 if Qt.Key_W in self.pressed_keys else 0x05
        left  = 0xA0 if Qt.Key_A in self.pressed_keys else 0x05
        down  = 0xA0 if Qt.Key_S in self.pressed_keys else 0x05
        right = 0xA0 if Qt.Key_D in self.pressed_keys else 0x05

        if self.stm32_comm.is_connected:
            self.stm32_comm.send_key_state(up, left, down, right)

    # ================= SERIAL =================
    def scan_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}")

    def connect_to_sender(self):
        port = self.port_combo.currentText().split(" - ")[0]
        baud = int(self.baud_combo.currentText())
        if self.stm32_comm.connect_to_sender(port, baud):
            self.stm32_comm.start()
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)

    def disconnect_from_sender(self):
        self.stm32_comm.disconnect_from_sender()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def update_connection_status(self, c):
        self.connection_label.setText("STM32: Connected" if c else "STM32: Disconnected")

    def handle_error(self, msg):
        QMessageBox.warning(self, "Error", msg)


def main():
    app = QApplication(sys.argv)
    window = RoverControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
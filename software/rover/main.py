"""
Entry point for rover control.

This module serves as the main control system for the search-and-rescue rover,
running on the Raspberry Pi or microcontroller. It coordinates all rover subsystems
and handles communication with the operator interface.

Key responsibilities:
- Initialize all rover subsystems
- Process incoming commands from operator interface
- Coordinate motor control, sensor readings, and camera operations
- Handle power management and safety systems
- Maintain wireless communication with operator interface
"""

from __future__ import annotations

import json
import socket
import time
from typing import Dict, Tuple

from motor_control import MotorController, MotorDriver


# Network configuration for receiving control commands
UDP_BIND_IP: str = "0.0.0.0"
UDP_BIND_PORT: int = 5005
MAX_PACKET_SIZE: int = 1024

# Control loop and safety configuration
UPDATE_RATE_HZ: float = 30.0
COMMAND_TIMEOUT_SEC: float = 0.7  # stop if no command received within this time


def _parse_command(data: bytes) -> Tuple[float, float]:
    """
    Parse incoming JSON bytes and extract (throttle, turn) in [-1, 1].
    Missing or invalid values fall back to zeros.
    """
    try:
        obj: Dict[str, float] = json.loads(data.decode("utf-8"))
        throttle = float(obj.get("throttle", 0.0))
        turn = float(obj.get("turn", 0.0))
        # Clamp to acceptable range
        throttle = max(-1.0, min(1.0, throttle))
        turn = max(-1.0, min(1.0, turn))
        return throttle, turn
    except Exception:
        return 0.0, 0.0


def _setup_udp_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_BIND_IP, UDP_BIND_PORT))
    sock.setblocking(False)
    return sock


def run() -> None:
    # Initialize motor control
    driver = MotorDriver()
    controller = MotorController(
        driver,
        pivot_mode="swing",  # "spin" for in-place turns
        default_update_rate_hz=UPDATE_RATE_HZ,
    )

    # Set up UDP receiver
    sock = _setup_udp_socket()
    print(f"[network] Listening for UDP JSON on {UDP_BIND_IP}:{UDP_BIND_PORT}")

    last_cmd_time = 0.0
    last_throttle = 0.0
    last_turn = 0.0

    dt = 1.0 / UPDATE_RATE_HZ
    try:
        while True:
            loop_start = time.time()

            # Drain any pending UDP packets (process the most recent)
            received_any = False
            while True:
                try:
                    data, _addr = sock.recvfrom(MAX_PACKET_SIZE)
                except BlockingIOError:
                    break
                throttle, turn = _parse_command(data)
                last_throttle, last_turn = throttle, turn
                last_cmd_time = loop_start
                received_any = True

            # Safety timeout: stop if no fresh command recently
            if (loop_start - last_cmd_time) > COMMAND_TIMEOUT_SEC:
                controller.stop()
            else:
                controller.set_command(last_throttle, last_turn, dt=dt)

            # Maintain update rate
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[main] Shutting down...")
    finally:
        controller.stop()
        try:
            sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    run()

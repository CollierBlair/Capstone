"""
Controls rover motors and movement logic.

This module handles all motor control operations for the rover, including
differential drive control, speed management, and safety features.

Key responsibilities:
- Control left and right motor speeds for differential drive
- Implement movement commands (forward, backward, turn, stop)
- Handle motor speed ramping and smooth acceleration
- Implement safety limits and emergency stop functionality
- Monitor motor status and report any issues
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


def _apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def _clamp(value: float, minimum: float, maximum: float) -> float:
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


@dataclass
class MotorOutputs:
    left: float
    right: float

    def as_tuple(self) -> Tuple[float, float]:
        return self.left, self.right


class MotorDriver:
    """
    Hardware abstraction to control left/right motor groups.

    Implement `apply(left, right)` to send normalized commands in [-1, 1]
    to the actual motor driver (e.g., L298N, TB6612FNG, or ESCs). A negative
    value indicates reverse. This default implementation is a stub that stores
    the last command; replace with GPIO/PWM code on the target platform.
    """

    def __init__(self) -> None:
        self._last_left: float = 0.0
        self._last_right: float = 0.0
        self._e_stopped: bool = False

    def apply(self, left: float, right: float) -> None:
        if self._e_stopped:
            self._last_left = 0.0
            self._last_right = 0.0
            return
        self._last_left = _clamp(left, -1.0, 1.0)
        self._last_right = _clamp(right, -1.0, 1.0)
        # TODO: Integrate actual GPIO/PWM writes for left/right groups.

    def stop(self) -> None:
        self.apply(0.0, 0.0)

    def emergency_stop(self) -> None:
        self._e_stopped = True
        self.stop()

    def clear_emergency_stop(self) -> None:
        self._e_stopped = False

    @property
    def last_command(self) -> MotorOutputs:
        return MotorOutputs(self._last_left, self._last_right)


class GpioMotorDriver(MotorDriver):
    """
    Raspberry Pi GPIO-based motor driver using one PWM pin and two direction
    pins per side. Requires an H-bridge/motor driver (e.g., TB6612FNG/L298N).

    Wiring per side (example):
    - left_dir1, left_dir2 -> driver IN1/IN2 (or AIN1/AIN2)
    - left_pwm -> driver ENA/PWMA
    - right_dir1, right_dir2 -> driver IN3/IN4 (or BIN1/BIN2)
    - right_pwm -> driver ENB/PWMB

    Note: This does NOT power motors directly. Use a separate motor supply.
    """

    def __init__(self,
                 left_dir1: int,
                 left_dir2: int,
                 left_pwm: int,
                 right_dir1: int,
                 right_dir2: int,
                 right_pwm: int,
                 pwm_frequency_hz: int = 1000) -> None:
        super().__init__()
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except Exception as exc:
            raise RuntimeError("RPi.GPIO is required on Raspberry Pi for GPIO PWM") from exc

        self._GPIO = GPIO
        self._GPIO.setmode(self._GPIO.BCM)

        self._left_dir1 = left_dir1
        self._left_dir2 = left_dir2
        self._left_pwm_pin = left_pwm
        self._right_dir1 = right_dir1
        self._right_dir2 = right_dir2
        self._right_pwm_pin = right_pwm

        for pin in (self._left_dir1, self._left_dir2, self._right_dir1, self._right_dir2,
                    self._left_pwm_pin, self._right_pwm_pin):
            self._GPIO.setup(pin, self._GPIO.OUT)

        self._left_pwm = self._GPIO.PWM(self._left_pwm_pin, pwm_frequency_hz)
        self._right_pwm = self._GPIO.PWM(self._right_pwm_pin, pwm_frequency_hz)
        self._left_pwm.start(0)
        self._right_pwm.start(0)

    def _set_side(self, magnitude: float, forward_pin: int, reverse_pin: int, pwm) -> None:  # type: ignore[no-redef]
        mag = _clamp(abs(magnitude), 0.0, 1.0)
        # Direction: True -> forward pin HIGH, False -> reverse pin HIGH
        forward = magnitude >= 0.0
        self._GPIO.output(forward_pin, self._GPIO.HIGH if forward else self._GPIO.LOW)
        self._GPIO.output(reverse_pin, self._GPIO.LOW if forward else self._GPIO.HIGH)
        pwm.ChangeDutyCycle(mag * 100.0)

    def apply(self, left: float, right: float) -> None:
        if self._e_stopped:
            self._left_pwm.ChangeDutyCycle(0.0)
            self._right_pwm.ChangeDutyCycle(0.0)
            self._GPIO.output(self._left_dir1, self._GPIO.LOW)
            self._GPIO.output(self._left_dir2, self._GPIO.LOW)
            self._GPIO.output(self._right_dir1, self._GPIO.LOW)
            self._GPIO.output(self._right_dir2, self._GPIO.LOW)
            self._last_left = 0.0
            self._last_right = 0.0
            return

        self._last_left = _clamp(left, -1.0, 1.0)
        self._last_right = _clamp(right, -1.0, 1.0)

        self._set_side(self._last_left, self._left_dir1, self._left_dir2, self._left_pwm)
        self._set_side(self._last_right, self._right_dir1, self._right_dir2, self._right_pwm)

    def stop(self) -> None:
        self.apply(0.0, 0.0)

    def cleanup(self) -> None:
        # Stop PWM and clean up GPIO
        try:
            self._left_pwm.stop()
            self._right_pwm.stop()
        finally:
            self._GPIO.cleanup()

class MotorController:
    """
    Differential control for tracked chassis with two sides (left/right).

    Inputs:
    - throttle: forward/backward command in [-1, 1]
    - turn: left/right command in [-1, 1] (positive turns right)

    Behavior:
    - Normal mixing: left = throttle + turn, right = throttle - turn
    - Pivot (one-side turn): when |throttle| is near 0, left = turn, right = -turn
    - Outputs are clamped to [-1, 1] and sent to the `MotorDriver`.
    """

    def __init__(self,
                 driver: MotorDriver,
                 deadband: float = 0.05,
                 pivot_deadband: float = 0.1,
                 pivot_mode: str = "swing",
                 max_accel_per_sec: float = 2.5,
                 default_update_rate_hz: float = 30.0) -> None:
        self._driver = driver
        self._deadband = max(0.0, min(0.2, deadband))
        self._pivot_deadband = max(0.0, min(0.3, pivot_deadband))
        self._pivot_mode = pivot_mode  # "swing" (one side) or "spin" (opposite)

        # Ramping configuration
        self._max_accel_per_sec = max(0.1, max_accel_per_sec)
        self._default_dt = 1.0 / max(1.0, default_update_rate_hz)

        # Track last commanded (post-ramp) outputs for slew limiting
        self._last_left: float = 0.0
        self._last_right: float = 0.0

    def _slew_limit(self, target: float, current: float, dt: float) -> float:
        max_delta = self._max_accel_per_sec * dt
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def set_command(self, throttle: float, turn: float, dt: float | None = None) -> MotorOutputs:
        throttle = _clamp(_apply_deadband(throttle, self._deadband), -1.0, 1.0)
        turn = _clamp(_apply_deadband(turn, self._deadband), -1.0, 1.0)
        dt = self._default_dt if dt is None else max(1e-3, dt)

        if abs(throttle) <= self._pivot_deadband and abs(turn) > 0.0:
            if self._pivot_mode == "spin":
                # Spin-in-place: opposite directions
                left = turn
                right = -turn
            else:
                # Swing turn: one side moves, the other stops
                if turn > 0.0:
                    # turn right: drive left side forward
                    left = abs(turn)
                    right = 0.0
                else:
                    # turn left: drive right side forward
                    left = 0.0
                    right = abs(turn)
        else:
            # Differential mixing for moving while turning.
            left = throttle + turn
            right = throttle - turn

        # Normalize to keep within [-1, 1] if mixing overflowed
        max_mag = max(1.0, abs(left), abs(right))
        left /= max_mag
        right /= max_mag

        # Apply slew rate limiting for smooth acceleration
        left = self._slew_limit(left, self._last_left, dt)
        right = self._slew_limit(right, self._last_right, dt)

        # Clamp again after slew limiting
        left = _clamp(left, -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)

        self._last_left = left
        self._last_right = right

        self._driver.apply(left, right)
        return MotorOutputs(left, right)

    def stop(self) -> None:
        self._driver.stop()

    def emergency_stop(self) -> None:
        self._driver.emergency_stop()

    def clear_emergency_stop(self) -> None:
        self._driver.clear_emergency_stop()


__all__ = [
    "MotorOutputs",
    "MotorDriver",
    "GpioMotorDriver",
    "MotorController",
]

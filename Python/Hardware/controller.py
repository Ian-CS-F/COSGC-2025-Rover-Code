"""
controller.py
Direct left/right motor control over USB serial to the Arduino Mega.

Usage:
    import controller
    ser = controller.connect()
    controller.set_motors(ser, left=0.6, right=0.6)   # forward
    controller.set_motors(ser, left=-0.5, right=0.5)  # spin left
    controller.set_motors(ser, left=0.0, right=0.0)   # stop
    controller.disconnect(ser)

Speed range: -1.0 (full reverse) to +1.0 (full forward) per side.
Uses SYS_DRIVE (5) command — true per-side PWM, no distance tracking.
"""

import time
import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 9600

_SYS_DRIVE = 5


def connect(port: str = SERIAL_PORT, baud: int = BAUD_RATE) -> serial.Serial:
    """Open serial and complete READY/ACK handshake. Returns the Serial object."""
    ser = serial.Serial(port, baud, timeout=1)
    ser.reset_input_buffer()
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line == "READY":
            ser.write(b"ACK\n")
            return ser
    raise RuntimeError("Arduino did not send READY within 10s")


def disconnect(ser: serial.Serial) -> None:
    """Stop motors and close the port."""
    set_motors(ser, 0.0, 0.0)
    ser.close()


def set_motors(ser: serial.Serial, left: float, right: float) -> None:
    """
    Set left and right motor speeds.

    Args:
        left:  -1.0 (full reverse) to +1.0 (full forward)
        right: -1.0 (full reverse) to +1.0 (full forward)
    """
    left  = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    ser.write(f"({_SYS_DRIVE},0,{left:.3f},{right:.3f})\n".encode())


def stop(ser: serial.Serial) -> None:
    set_motors(ser, 0.0, 0.0)

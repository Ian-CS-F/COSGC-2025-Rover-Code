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


if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    print(f"Connecting on {port}...")
    ser = connect(port)
    print("Connected. Commands: w/s=forward/back, a/d=turn, space=stop, q=quit")
    print("Speed up/down: +/-")

    speed = 0.5

    try:
        import msvcrt  # Windows
        def get_key():
            return msvcrt.getch().decode(errors="replace").lower()
    except ImportError:
        import tty, termios  # Linux/Mac
        def get_key():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                return sys.stdin.read(1).lower()
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

    try:
        while True:
            key = get_key()
            if key == "w":
                set_motors(ser, speed, speed)
                print(f"Forward  {speed:.1f}")
            elif key == "s":
                set_motors(ser, -speed, -speed)
                print(f"Reverse  {speed:.1f}")
            elif key == "a":
                set_motors(ser, -speed, speed)
                print(f"Turn left  {speed:.1f}")
            elif key == "d":
                set_motors(ser, speed, -speed)
                print(f"Turn right  {speed:.1f}")
            elif key == " ":
                stop(ser)
                print("Stop")
            elif key in ("+", "="):
                speed = min(1.0, round(speed + 0.1, 1))
                print(f"Speed → {speed:.1f}")
            elif key == "-":
                speed = max(0.1, round(speed - 0.1, 1))
                print(f"Speed → {speed:.1f}")
            elif key == "q":
                break
    finally:
        disconnect(ser)
        print("Disconnected.")

#!/usr/bin/env python3
"""
Shutdown button monitor
=======================
GPIO 17 (Pin 11) — button to GND (Pin 9).
Hold for DEBOUNCE_S seconds to trigger a clean shutdown.

Designed to run as a systemd service (shutdown-button.service).
Does not interact with main.py or any other rover service.
"""

import logging
import subprocess
import time

import RPi.GPIO as GPIO

# ── Config ────────────────────────────────────────────────────────────────────
BUTTON_PIN  = 17      # BCM numbering — physical Pin 11
DEBOUNCE_S  = 2.0     # seconds button must be held before shutdown triggers
POLL_INTERVAL_S = 0.05  # how often to check the pin (50 ms)

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)s  %(message)s",
)
log = logging.getLogger("shutdown_button")

# ── GPIO setup ────────────────────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# Button wired to GND → pin reads LOW when pressed

log.info("Shutdown button monitor started (GPIO %d, debounce %.1f s).",
         BUTTON_PIN, DEBOUNCE_S)

# ── Main loop ─────────────────────────────────────────────────────────────────
press_start: float | None = None

try:
    while True:
        pressed = not GPIO.input(BUTTON_PIN)   # LOW = pressed

        if pressed:
            if press_start is None:
                press_start = time.monotonic()
                log.info("Button pressed — hold for %.1f s to shut down.", DEBOUNCE_S)
            elif time.monotonic() - press_start >= DEBOUNCE_S:
                log.info("Debounce elapsed — initiating shutdown now.")
                subprocess.run(["sudo", "shutdown", "now"], check=False)
                # Script will be terminated by systemd as the OS shuts down.
                # Sleep here so the log message is flushed before we exit.
                time.sleep(5)
        else:
            if press_start is not None:
                held = time.monotonic() - press_start
                log.info("Button released after %.2f s — shutdown cancelled.", held)
            press_start = None

        time.sleep(POLL_INTERVAL_S)

except KeyboardInterrupt:
    log.info("Interrupted — exiting.")
finally:
    GPIO.cleanup()

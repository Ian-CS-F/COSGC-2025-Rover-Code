# Raspberry Pi Setup Instructions

Complete setup guide for the COSGC 2025 rover Pi.
Follow these steps in order after a fresh flash.

---

## 1. Flash the SD Card

Use **Raspberry Pi Imager** (raspberrypi.com/software).

- OS: **Raspberry Pi OS Lite (64-bit)**
- Click the **settings gear** before writing and configure:
  - Hostname: `COSGC2026Rover`
  - Username: `cosgc_26_rover`
  - Password: *(your password)*
  - WiFi SSID and password: *(your hotspot)*
  - **Enable SSH**

---

## 2. First Boot — SSH In

Wait ~60 seconds after powering on, then:

```bash
ssh cosgc_26_rover@COSGC2026Rover.local
```

If `.local` doesn't resolve, find the IP from your router/hotspot device list and use that directly.

---

## 3. Check overlayroot is Disabled

overlayroot causes all changes to be lost on reboot. Check it is off:

```bash
mount | grep "on / "
```

If the output contains `ext4 (rw` — you are fine, skip to step 4.

If it contains `overlay` — overlayroot is active. Disable it:

```bash
cat /boot/firmware/cmdline.txt
```

If you see `overlayroot=tmpfs` in that line, remove it:

```bash
sudo nano /boot/firmware/cmdline.txt
```

Use **Ctrl+\\** (replace), search for `overlayroot=tmpfs ` (with trailing space), replace with nothing, confirm with `y`.
Save with **Ctrl+O**, Enter, **Ctrl+X**, then reboot:

```bash
sudo reboot
```

SSH back in and re-run the mount check to confirm it now shows `ext4 (rw`.

---

## 4. Install Dependencies

Update the system first:

```bash
sudo apt update && sudo apt upgrade -y
```

Install system packages:

```bash
sudo apt install -y python3-pip python3-smbus i2c-tools git
```

Install Python packages:

```bash
pip3 install pyserial smbus2 RPi.GPIO --break-system-packages
```

> Note: `--break-system-packages` is required on Raspberry Pi OS Bookworm (2023+).
> If you get a different error try without that flag.

Enable I2C:

```bash
sudo raspi-config
```

Go to **Interface Options → I2C → Enable**, then **Finish**.

Reboot to apply I2C changes:

```bash
sudo reboot
```

SSH back in, then verify I2C is working (should list your devices):

```bash
i2cdetect -y 1
```

---

## 5. Clone the Repository

```bash
cd ~
git clone https://github.com/<your-repo>/COSGC-2025-Rover-Code.git
```

*(Replace with your actual GitHub URL.)*

---

## 6. Install the Systemd Services

**Important:** Plug the Arduino into the Pi via USB before starting rover-main, otherwise it won't start (this is intentional — it prevents a crash loop).

From `~/COSGC-2025-Rover-Code`:

```bash
sudo cp PiSetup/rover-main.service /etc/systemd/system/rover-main.service
sudo cp PiSetup/shutdown-button.service /etc/systemd/system/shutdown-button.service
sudo systemctl daemon-reload
sudo systemctl enable rover-main.service shutdown-button.service
sudo systemctl start shutdown-button.service
```

Do **not** manually start rover-main — it will start automatically when the Arduino is plugged in.

---

## 7. Verify Services

```bash
sudo systemctl status shutdown-button.service
sudo systemctl status rover-main.service
```

- `shutdown-button` should show `active (running)`
- `rover-main` should show `active (running)` if Arduino is plugged in, or `inactive` if not — both are fine

View live logs:

```bash
journalctl -fu rover-main.service
journalctl -fu shutdown-button.service
```

---

## 8. Reboot Test

```bash
sudo reboot
```

SSH back in after ~60 seconds and re-run the status checks to confirm both services come back up automatically.

---

## Hardware Notes

| Item | Detail |
|---|---|
| Arduino serial port | `/dev/ttyACM0` |
| I2C bus | 1 (GPIO 2/3, pins 3/5) |
| Shutdown button | GPIO 17 (pin 11) → GND (pin 9), hold 2 seconds |
| Tilt servo primary | Pin 12 |
| Tilt servo secondary | Pin 13 (mirror-mounted, moves inverse) |
| Left encoder A | Pin 2 (INT0) |
| Left encoder B | Pin 10 |
| Right encoder A | Pin 3 (INT1) |
| Right encoder B | Pin 11 |

---

## Common Problems

**Can't SSH in / Pi not on network**
- Wait 2 minutes after power on
- Make sure the shutdown button switch is OPEN before booting — if it is closed the Pi will shut down 2 seconds after boot
- Check your hotspot device list for the IP

**rover-main not starting**
- Arduino must be plugged in via USB — the service waits for `/dev/ttyACM0`
- Check with `sudo systemctl status rover-main.service`

**Changes lost after reboot**
- overlayroot is active — follow step 3 above

**Green activity light stays solid**
- A service is crash-looping — run `sudo systemctl stop rover-main.service` immediately then diagnose with `journalctl -u rover-main.service -n 30 --no-pager`

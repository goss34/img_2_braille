**`brltty`** can interfere with serial communication to the Canute 360 because it automatically binds to USB-connected Braille devices, including the Canute, effectively locking the serial port.

### ✅ To fix this, you usually need to:

1. **Uninstall or disable `brltty`:**

   ```bash
   sudo apt remove brltty
   sudo apt purge brltty  # optional: removes config files too
   ```

2. **Then reboot** or **unplug and reconnect** the Canute 360.

After this, the Canute 360 should appear as a regular serial device (e.g., `/dev/ttyACM0` or `/dev/ttyUSB0`) and be accessible for serial communication via your custom scripts.

Let me know if you want a script to check for `brltty` and warn if it's running.

---

### ✅ Python Packages

```bash
sudo apt update
sudo apt install python3-pip

pip install pyserial
pip3 install opencv-python
pip3 install crcmod

```

---

### ✅ Add your user to the `dialout` group to use the serial port.

This is the standard solution on Ubuntu and similar Linux distros:

```bash
sudo usermod -a -G dialout $USER
```

Then **log out and log back in**, or reboot, to apply the group change.

---
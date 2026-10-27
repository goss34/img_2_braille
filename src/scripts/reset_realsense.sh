#!/bin/bash
echo "Resetting RealSense camera..."
DEVICE_PATH=$(lsusb | grep 8086:0b5c | awk '{print "/dev/bus/usb/" $2 "/" $4}' | sed 's/://')
if [ -z "$DEVICE_PATH" ]; then
    echo "RealSense not found!"
    exit 1
fi
usbreset "$DEVICE_PATH"
echo "Reset complete."
#!/bin/bash

# Script to start BRLTTY for Canute display
# Exit on any error
set -e

echo "Starting BRLTTY initialization..."

# Check if script is run with sudo
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo"
    exit 1
fi

# Stop any existing BRLTTY processes
echo "Stopping any existing BRLTTY processes..."
pkill brltty 2>/dev/null || true  # Don't fail if no processes found

# Remove existing BrlAPI socket
echo "Cleaning up old BrlAPI socket..."
rm -f /var/lib/BrlAPI/.0

# Start BRLTTY with Canute configuration
echo "Starting BRLTTY..."
brltty -b cn -d usb: -x a2 -n

# Script will exit here when BRLTTY is stopped
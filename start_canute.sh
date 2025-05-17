#!/bin/bash

# BRLTTY Startup Script
# This script terminates any running BRLTTY instances and starts a new configuration
# with Canute support for console and AtSpi2 support for X11

# Location of BRLTTY repository
BRLTTY_PATH="$HOME/Documents/brltty"

# Print status messages
echo "Stopping any running BRLTTY instances..."

# Safely terminate any existing BRLTTY processes
ps aux | grep '[b]rltty' | awk '{print $2}' | xargs -r kill -9
ps aux | grep '[b]rlapi' | awk '{print $2}' | xargs -r kill -9

# Give processes time to terminate
sleep 1

# Verify BRLTTY directory exists
echo "Changing to BRLTTY directory: $BRLTTY_PATH"
if [ ! -d "$BRLTTY_PATH" ]; then
    echo "Error: BRLTTY directory not found at $BRLTTY_PATH"
    echo "Please update the BRLTTY_PATH variable in this script."
    exit 1
fi

# Change to BRLTTY directory
cd "$BRLTTY_PATH" || {
    echo "Error: Could not change to directory $BRLTTY_PATH"
    exit 1
}

# Verify run-brltty script exists and is executable
if [ ! -x "./run-brltty" ]; then
    echo "Error: run-brltty script not found or not executable in $BRLTTY_PATH"
    ls -la ./run-brltty 2>/dev/null || echo "File does not exist"
    exit 1
fi

# Start the primary BRLTTY instance for Canute over USB
echo "Starting BRLTTY with Canute driver..."
./run-brltty -b cn -d usb: &
FIRST_PID=$!
echo "First BRLTTY instance started with PID: $FIRST_PID"

# Wait for the first instance to initialize
echo "Waiting for primary instance to initialize..."
sleep 3

# Start the secondary BRLTTY instance for X11 support
echo "Starting BRLTTY with BrlAPI and AtSpi2 support..."
./run-brltty -b ba -x a2 &
SECOND_PID=$!
echo "Second BRLTTY instance started with PID: $SECOND_PID"

echo "BRLTTY setup complete."
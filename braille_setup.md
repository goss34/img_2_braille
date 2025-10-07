# Setting up custom ROS package to interact with nvblox and Canute 360

## Downloading and Placing Files

1. Grab files from the `src` directory in the repository

This should include the following files/directories:
- braille_interface
- scripts
- Dockerfile.braille

2. Navigate to the workspace directory inside of `nova_ssd`, ideally using a graphical file explorer like VSCode or the default Ubuntu file explorer works

3. Place `Dockerfile.braille` into the following directory:
```bash
${ISAAC_ROS_WS}/src/isaac_ros_common/docker/

# So that the structure is:
${ISAAC_ROS_WS}/src/isaac_ros_common/docker/Dockerfile.braille
```

4. Place `braille_interface` into the following directory:
```bash
${ISAAC_ROS_WS}/src/isaac_ros_common/

# So that the structure is:
${ISAAC_ROS_WS}/src/isaac_ros_common/braille_interface
```

5. Place `scripts` into the following directory:
```bash
${ISAAC_ROS_WS}/src/

# So that the structure is:
${ISAAC_ROS_WS}/src/scripts/reset_realsense.sh
```

6. Fix `reset_realsense.sh` by using the identifier of your unique physical device:
```bash
lsusb | grep "Intel"

# Output:
Bus 002 Device 003: ID 8086:0b5c Intel Corp. Intel(R) RealSense(TM) Depth Camera 455
```
Take note of the values after ID, in this case `8086:0b5c`, plug this value into the `reset_realsense` script here:
```shell
DEVICE_PATH=$(lsusb | grep 8086:0b5c | awk '{print "/dev/bus/usb/" $2 "/" $4}' | sed 's/://')
```
Save this updated script

7. Modify Docker build file to include `Dockerfile.braille`:
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/
```
Edit `.isaac_ros_common-config` so that the file looks like this:
```bash
CONFIG_IMAGE_KEY=ros2_humble.realsense.braille
```

8. Run container:
```bash
cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh
```

9. Build and source workspace:
```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

10. Navigate to `braille_interface` directory and run the project:
```bash
cd /workspaces/isaac_ros-dev/src/isaac_ros_common/braille_interface/launch
ros2 launch braille.launch.py
```

## Notes

When launching the container again after exiting, it is still mandatory to source the workspace every time:
```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
```

## Troubleshooting:

If you run into the following warning when attempting to run the braille project:

```bash
[component_container_mt-6] [INFO] [1759788685.356144476] [nvblox_node]: Tried to clear map outside of radius but couldn't look up frame: camera0_link
[component_container_mt-6] [INFO] [1759788685.886323810] [nvblox_node]: Lookup transform failed for frame camera0_link. Layer pointclouds not published
```

Try disconnecting and re-connecting the USB-C cable connected to the camera or the Jetson. It is required to use a USB-C cable that has USB 3.2 speeds.
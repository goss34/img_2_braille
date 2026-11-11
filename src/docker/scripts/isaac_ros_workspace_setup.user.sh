#!/bin/bash
#
# Isaac ROS Workspace Setup - Entrypoint Addition
# This script runs automatically when the container starts AS THE USER
#
# It does not build the workspace. Building at every start was slow, needed the internet
# for rosdep, and could rebuild nvblox unattended. Build by hand when needed:
#   build_braille   just the braille_interface package (the usual case)
#   build_ws        the whole workspace, including nvblox
#

if [ ! -f "${ISAAC_ROS_WS}/install/setup.bash" ]; then
    echo "======================================"
    echo "The Isaac ROS workspace hasn't been built yet."
    echo "Build it with: build_ws"
    echo "======================================"
fi

# Handle USB device permissions for Braille board
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 666 /dev/ttyACM0 2>/dev/null || true
fi

# Add convenience aliases to user's bashrc (only add once)
if ! grep -q "Isaac ROS convenience aliases" ${HOME}/.bashrc 2>/dev/null; then
    cat >> ${HOME}/.bashrc << 'EOF'

# Isaac ROS convenience aliases
alias build_ws='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --packages-skip nvblox_image_padding'
alias build_braille='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --packages-select braille_interface'
alias source_ws='source ${ISAAC_ROS_WS}/install/setup.bash'
alias launch_braille='cd ${ISAAC_ROS_WS}/src/isaac_ros_common/braille_interface/launch && ros2 launch braille.launch.py'

# Auto-source workspace on new shells
if [ -f "${ISAAC_ROS_WS}/install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    source ${ISAAC_ROS_WS}/install/setup.bash
fi
EOF
fi

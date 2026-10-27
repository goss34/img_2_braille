#!/bin/bash
#
# Isaac ROS Workspace Setup - Entrypoint Addition
# This script runs automatically when the container starts AS THE USER
#

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Check if workspace needs to be built
BUILD_MARKER="${HOME}/.isaac_ros_built"

if [ ! -f "${BUILD_MARKER}" ] && [ -d "${ISAAC_ROS_WS}/src" ]; then
    echo "======================================"
    echo "Building Isaac ROS workspace (first run)..."
    echo "This may take a few minutes..."
    echo "======================================"
    
    cd ${ISAAC_ROS_WS}
    
    # Update rosdep and install dependencies
    sudo rosdep update 2>/dev/null || true
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble 2>/dev/null || true
    
    # Build with proper flags to handle package overrides
    colcon build --symlink-install \
        --packages-skip nvblox_image_padding \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        touch "${BUILD_MARKER}"
        echo "======================================"
        echo "Workspace build complete!"
        echo "======================================"
    else
        echo "======================================"
        echo "Workspace build had errors, but continuing..."
        echo "You can rebuild manually with: build_ws"
        echo "======================================"
    fi
fi

# Source the workspace if it exists and is built
if [ -f "${ISAAC_ROS_WS}/install/setup.bash" ]; then
    source ${ISAAC_ROS_WS}/install/setup.bash
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
alias source_ws='source ${ISAAC_ROS_WS}/install/setup.bash'
alias launch_braille='cd ${ISAAC_ROS_WS}/src/isaac_ros_common/braille_interface/launch && ros2 launch braille.launch.py'

# Auto-source workspace on new shells
if [ -f "${ISAAC_ROS_WS}/install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    source ${ISAAC_ROS_WS}/install/setup.bash
fi
EOF
fi
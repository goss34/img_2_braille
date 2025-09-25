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

6. Modify Docker build file to include `Dockerfile.braille`:
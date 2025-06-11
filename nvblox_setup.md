# Setting up isaac_ros_nvblox for use on Jetson Orin

## [Setup Developer Environment](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)

1. Restart Docker:
```bash
sudo systemctl daemon-reload && sudo systemctl restart docker
```

2. Install Git LFS:
```bash
sudo apt install git-lfs
```
```bash
git lfs install --skip-repo
```

3. Create the ROS 2 workspace:
```bash
sudo mkdir -p /mnt/nova_ssd/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=/mnt/nova_ssd/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

**Note:** you may need to change ownership of the workspace location before continuing:
```bash
sudo chown -R $USER /mnt/nova_ssd
```

## [Isaac ROS RealSense Setup](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html)

1. Plug in the RealSense camera

2. Clone the isaac_ros_common repo:
```bash
cd ${ISAAC_ROS_WS}/src
sudo git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```

3. Configure the container to include RealSense packages:
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
```

4. Launch the Docker container:
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

**Note** If this fails for any reason, you may just need to run the ```run_dev.sh``` script again.

This will rebuild the container and may take a few minutes.

## [Jetson Setup for VPI](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_vpi.html)

1. Generate CDI Spec:
```bash
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
```

2. Install ```pva-allow-2```:
```bash
sudo apt update
sudo apt install software-properties-common
sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt update
sudo apt install -y pva-allow-2
```

## Test the Docker Container & RealSense Viewer

1. Navigate to the directory and launch the Docker container:
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

2. Once the Docker container is loaded, run ```realsense-viewer```:
```bash
realsense-viewer
```

This will bring up the RealSense Viewer application. If not already, connect the RealSense camera to the Jetson. It should either automatically load one of the sensors, otherwise three options will be presented to launch the depth, color or motion sensors.

* Note: ensure that you are using a USB 3.2 cable between the camera and the Jetson, dropped frames will occur if a USB 2 cable is used instead.

## [nvblox Installation & Setup](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#download-quickstart-assets)

1. Install dependencies:
```bash
sudo apt install -y curl jq tar
```

2. Set these variables and run these commands:
```bash
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_nvblox"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```

3. Launch Docker container:
```bash
cd $ISAAC_ROS_WS && ./src/isaac_ros_common/scripts/run_dev.sh
```

4. Install nvblox from Debian:
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-nvblox
rosdep update
rosdep install isaac_ros_nvblox
```

5. Run the example file to verify installation:
```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py \
rosbag:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_nvblox/quickstart \
navigation:=False
```


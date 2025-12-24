#!/bin/bash

# Color definitions
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Default to ROS1
ROS_VERSION=${1:-1}

if [ "$ROS_VERSION" != "1" ] && [ "$ROS_VERSION" != "2" ]; then
    echo -e "${RED}Invalid ROS version. Use: $0 [1|2]${NC}"
    echo -e "${YELLOW}Example:${NC}"
    echo -e "  $0 1    # Run ROS1 (default)"
    echo -e "  $0 2    # Run ROS2"
    exit 1
fi

if [ "$ROS_VERSION" = "1" ]; then
    IMAGE="ii-nvm:ros1"
    DOCKERFILE="Dockerfile.ros1"
    WS_DIR="/home/catkin_ws"
    CONTAINER_NAME="ii-nvm-ros1"
    LAUNCH_CMD="roslaunch II_NVM run.launch"
    BAG_CMD="rosbag play /bags/your.bag --clock"
    ROS_ENV=(
        "-e" "ROS_MASTER_URI=http://localhost:11311"
        "-e" "ROS_IP=127.0.0.1"
        "-e" "ROS_HOSTNAME=localhost"
    )
else
    IMAGE="ii-nvm:ros2"
    DOCKERFILE="Dockerfile.ros2"
    WS_DIR="/home/cc/ros2_ws"
    CONTAINER_NAME="ii-nvm-ros2"
    LAUNCH_CMD="ros2 launch II_NVM run.launch.py"
    BAG_CMD="ros2 bag play /bags/your.bag --clock"
    ROS_ENV=(
        "-e" "ROS_DOMAIN_ID=0"
        "-e" "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    )
fi

echo -e "${GREEN}=== II-NVM Docker Run Script (ROS${ROS_VERSION}) ===${NC}\n"

# Check if image exists
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo -e "${RED}Error: Image '$IMAGE' not found!${NC}"
    echo -e "${YELLOW}Please build first:${NC}"
    echo -e "  docker compose build ii-nvm-ros${ROS_VERSION}"
    echo -e "  or"
    echo -e "  docker build -f ${DOCKERFILE} -t ${IMAGE} ."
    exit 1
fi

# Setup X11 permissions
echo -e "${YELLOW}Setting up X11 permissions...${NC}"
xhost +local:docker > /dev/null 2>&1

# Create data directories
mkdir -p ./data ./bags

# Docker run options
DOCKER_OPTS=(
    "-it"
    "--rm"
    "--privileged"
    "--network=host"
    "--name=${CONTAINER_NAME}"
    "-e" "DISPLAY=${DISPLAY}"
    "-e" "QT_X11_NO_MITSHM=1"
    "${ROS_ENV[@]}"
    "-v" "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    "-v" "$(pwd)/data:/data"
    "-v" "$(pwd)/bags:/bags"
    "-w" "${WS_DIR}"
)

echo -e "${GREEN}Starting Docker container...${NC}\n"
echo -e "${YELLOW}Usage:${NC}"
echo -e "  1. ${LAUNCH_CMD}"
echo -e "  2. ${BAG_CMD}\n"

docker run "${DOCKER_OPTS[@]}" "${IMAGE}" bash

# Cleanup X11 permissions on exit
echo -e "\n${YELLOW}Cleaning up X11 permissions...${NC}"
xhost -local:docker > /dev/null 2>&1

echo -e "${GREEN}Done!${NC}"

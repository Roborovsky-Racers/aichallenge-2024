#!/bin/bash

sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
docker run --rm \
    --net=host \
    -e ROS_DISTRO=humble \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///vehicle/cyclonedds.xml \
    -v "${SCRIPT_DIR}:/vehicle" \
    --name zenoh \
    eclipse/zenoh-bridge-ros2dds:latest -c /vehicle/zenoh.json5

#!/bin/bash

IP_ADDR="192.168.1.68"

sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
docker run --rm \
    --net=host \
    -e ROS_DISTRO=humble \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///vehicle/cyclonedds.xml \
    -v "${SCRIPT_DIR}/../vehicle:/vehicle" \
    --name zenoh \
    eclipse/zenoh-bridge-ros2dds:latest -e "tcp/${IP_ADDR}:7447"
#!/bin/bash
pkill -9 rviz2
ros2 run rviz2 rviz2 -d /aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/config/autoware.rviz.custom -s $(find-pkg-share autoware_launch)/rviz/image/autoware.png

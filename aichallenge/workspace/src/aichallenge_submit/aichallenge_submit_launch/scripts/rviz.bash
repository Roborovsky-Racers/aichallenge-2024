#!/bin/bash
pkill -9 rviz2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix aichallenge_submit_launch)/share/aichallenge_submit_launch/config/autoware.rviz.custom

#!/bin/bash

# build Autoware
cd /aichallenge
./build_autoware.bash

# build autoware-practice
cd /autoware-practice
vcs import src < autoware.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
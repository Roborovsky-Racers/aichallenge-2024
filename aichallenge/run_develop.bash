#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM

# Move working directory
OUTPUT_DIRECTORY=$(date +%Y%m%d-%H%M%S)
cd /output || exit
mkdir "$OUTPUT_DIRECTORY"
ln -nfs "$OUTPUT_DIRECTORY" latest
cd "$OUTPUT_DIRECTORY" || exit

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

# Start AWSIM
echo "Start AWSIM"
$AWSIM_DIRECTORY/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!

# Start Autoware
echo "Start Autoware"
# ros2 launch aichallenge_system_launch aichallenge_system.launch.xml >autoware.log 2>&1 &
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml rviz_config:=$(ros2 pkg prefix aichallenge_system_launch)/share/aichallenge_system_launch/config/autoware.rviz.custom >autoware.log 2>&1 &
PID_AUTOWARE=$!

sleep 5

# Start driving and wait for the simulation to finish
echo "Waiting for the simulation"
ros2 topic pub --once /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null
wait $PID_AWSIM

# Stop Autoware
echo "Stop Autoware"
kill $PID_AUTOWARE
wait $PID_AUTOWARE

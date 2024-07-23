sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM"
	@/aichallenge/simulator/AWSIM/AWSIM.x86_64 > /dev/null

gpu-sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM with GPU"
	@/aichallenge/simulator/AWSIM_GPU/AWSIM.x86_64 > /dev/null

rviz:
	@echo "Start RVIZ"
	ros2 run rviz2 rviz2 -d /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz.custom

aw:
	@echo "Start Auwoware"
	ros2 run aichallenge_system_launch object_marker.py &
	ros2 run aichallenge_system_launch control_mode_adapter.py &
	ros2 launch aichallenge_submit_launch aichallenge_submit.launch.xml

start:
	@echo "Start cotnrol"
	ros2 topic pub --once /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null

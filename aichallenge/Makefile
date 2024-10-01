RVIZ_CONFIG="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/debug_sensing.rviz"
# RVIZ_CONFIG="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz.custom"
RVIZ_IMAGE="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/rviz/image/autoware.png"

sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM"
	@${AWSIM_PATH}/AWSIM.x86_64 --timeout 86400.0 &

aw:
	@echo "Start Autoware"
	@ros2 launch aichallenge_system_launch aichallenge_system.launch.xml rviz_config:=$(RVIZ_CONFIG) >autoware.log 2>&1 &

mpc: start
	@echo "Start MPC"
	@ros2 launch multi_purpose_mpc_ros mpc_controller.launch.py

start: reset
	@echo "Start cotnrol"
	@ros2 service call /control/control_mode_request autoware_auto_vehicle_msgs/srv/ControlModeCommand '{mode: 1}' >/dev/null &

reset:
	@echo "Reset simulation"
	@ros2 topic pub --once /aichallenge/awsim/reset std_msgs/msg/Empty {} >/dev/null &
	@ros2 run multi_purpose_mpc_ros publish_initialpose

kill:
	@echo "Kill all"
	@bash -i -c "rkill"

setup: sim aw
	@echo ""


all: sim aw mpc
	@echo ""
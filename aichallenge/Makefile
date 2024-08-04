sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM"
	@${AWSIM_PATH}/AWSIM.x86_64 --timeout 86400.0 &

rviz:
	@echo "Start RVIZ"
	@ros2 run rviz2 rviz2 -d /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz.custom --ros-args -r rviz:/initialpose:=/localization/initial_pose3d &
	@ros2 run aichallenge_system_launch object_marker.py &

aw:
	@echo "Start Auwoware"
	@ros2 run aichallenge_system_launch control_mode_adapter.py &
	@ros2 run goal_pose_setter goal_pose_setter_node --ros-args --params-file $(shell ros2 pkg prefix --share goal_pose_setter)/config/default_goal_pose.param.yaml &
	@ros2 launch aichallenge_submit_launch reference.launch.xml vehicle_model:=racing_kart sensor_model:=racing_kart_sensor_kit map_path:=$(shell ros2 pkg prefix --share aichallenge_submit_launch)/map &

mpc: start
	@echo "Start MPC"
	@ros2 launch multi_purpose_mpc_ros mpc_controller.launch.py &

start:
	@echo "Start cotnrol"
	ros2 topic pub -t 10 /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null &

reset:
	@echo "Reset simulation"
	@ros2 topic pub --once /aichallenge/awsim/reset std_msgs/msg/Empty {} >/dev/null &
	@ros2 run multi_purpose_mpc_ros publish_initialpose

kill:
	@echo "Kill all"
	@bash -i -c "rkill"

setup: sim rviz aw

all: sim rviz aw mpc
# RVIZ_CONFIG="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/debug_sensing.rviz"
RVIZ_CONFIG="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/debug_sensing.rviz.custom"
RVIZ_IMAGE="/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/rviz/image/autoware.png"
MAP_YAML_FILE="/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/maps/occupancy_grid_map.yaml"

sim:
	@sudo ip link set multicast on lo
	@sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
	@echo "Start AWSIM"
	@${AWSIM_PATH}/AWSIM.x86_64 --timeout 86400.0 &

rviz:
	@ros2 run rviz2 rviz2 -d $(RVIZ_CONFIG) -s $(RVIZ_IMAGE)

aw:
	@echo "Start Autoware in SIM"
	@ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=true use_sim_time:=true run_rviz:=true rviz_config:=$(RVIZ_CONFIG) >autoware.log 2>&1 &

vehicle:
	@echo "Start Autoware in REAL"
	@sudo ip link set multicast on lo
	@ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=false run_rviz:=false rviz_config:=$(RVIZ_CONFIG)

mpc: start
	@echo "Start MPC"
	@ros2 launch multi_purpose_mpc_ros mpc_controller.launch.py use_sim_time:=true use_boost_acceleration:=false use_obstacle_avoidance:=false use_stats:=true

mpc-real: start
	@echo "Start MPC"
	@ros2 launch multi_purpose_mpc_ros mpc_controller.launch.py use_sim_time:=false use_boost_acceleration:=false use_obstacle_avoidance:=false use_stats:=true

start: reset
	@echo "Start cotnrol"
	@ros2 service call /control/control_mode_request autoware_auto_vehicle_msgs/srv/ControlModeCommand '{mode: 1}' >/dev/null &

stop:
	@echo "Request Stop!"
	@ros2 topic pub /control/mpc/stop_request std_msgs/msg/Empty {} -r 10 -t 30

restart:
	@echo "Request Restart"
	@ros2 topic pub /control/control_mode_request_topic std_msgs/msg/Bool "data: true" -r 10 -t 20

reset:
	@echo "Reset simulation and publish initialpose"
	@ros2 topic pub --once /aichallenge/awsim/reset std_msgs/msg/Empty {} >/dev/null &
	@ros2 run multi_purpose_mpc_ros publish_initialpose

kill:
	@echo "Kill all"
	@bash -i -c "rkill"

setup: sim aw
	@echo ""

map:
	@echo "Start map server"
	@ros2 launch aichallenge_submit_launch map_server.launch.py map_yaml_file:=$(MAP_YAML_FILE)

record:
	@echo "Start rosbag recording"
	@mkdir -p rosbag2_autoware
	@cd rosbag2_autoware && ros2 bag record -a

all: sim aw mpc
	@echo ""
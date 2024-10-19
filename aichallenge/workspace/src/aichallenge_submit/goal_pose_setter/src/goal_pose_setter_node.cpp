#include "goal_pose_setter_node.hpp"

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher")
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    ekf_trigger_client_ = this->create_client<std_srvs::srv::SetBool>("/localization/trigger_node");
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", qos);
    gear_command_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", qos);
    route_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        "/planning/mission_planning/route_state",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        std::bind(&GoalPosePublisher::route_state_callback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 1,
        std::bind(&GoalPosePublisher::odometry_callback, this, std::placeholders::_1));

    pit_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/aichallenge/pitstop/area", 1,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            pit_position_.position.x = msg->data[0];
            pit_position_.position.y = msg->data[1];
            pit_position_.position.z = msg->data[2];
            pit_position_.orientation.x = msg->data[3];
            pit_position_.orientation.y = msg->data[4];
            pit_position_.orientation.z = msg->data[5];
            pit_position_.orientation.w = msg->data[6];
        });

    pit_condition_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/aichallenge/pitstop/condition", 1,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            pit_condition_ = msg->data;
        });

    pit_stop_time_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/aichallenge/pitstop/status", 1,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            pit_stop_time_ = msg->data;
        });

    reset_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
        "/aichallenge/awsim/reset", 1,
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            is_started_ = false;
            current_goal_state_ = CurrentGoalState::NONE;
            stop_initializing_pose_ = false;
            stop_streaming_goal_pose_ = false;
            pit_condition_ = 0;
            pit_stop_time_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Reset goal pose");
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(300),
        std::bind(&GoalPosePublisher::on_timer, this));

    this->declare_parameter("goal.position.x", 21920.2);
    this->declare_parameter("goal.position.y", 51741.1);
    this->declare_parameter("goal.position.z", 0.0);
    this->declare_parameter("goal.orientation.x", 0.0);
    this->declare_parameter("goal.orientation.y", 0.0);
    this->declare_parameter("goal.orientation.z", 0.645336);
    this->declare_parameter("goal.orientation.w", 0.763899);

    this->declare_parameter("half_goal.position.x", 89657.0);
    this->declare_parameter("half_goal.position.y", 43175.0);
    this->declare_parameter("half_goal.position.z", -28.0);
    this->declare_parameter("half_goal.orientation.x", 0.0);
    this->declare_parameter("half_goal.orientation.y", 0.0);
    this->declare_parameter("half_goal.orientation.z", -0.9);
    this->declare_parameter("half_goal.orientation.w", 0.25);

    this->declare_parameter("pit_goal.position.x", 89626.3671875);
    this->declare_parameter("pit_goal.position.y", 43134.921875);
    this->declare_parameter("pit_goal.position.z", 42.10000228881836);
    this->declare_parameter("pit_goal.orientation.x", 0.0);
    this->declare_parameter("pit_goal.orientation.y", 0.0);
    this->declare_parameter("pit_goal.orientation.z", -0.8788172006607056);
    this->declare_parameter("pit_goal.orientation.w", -0.47715866565704346);

    this->declare_parameter("goal_range", 10.0);
    this->declare_parameter("pit_in_range", 3.0);

    this->declare_parameter("enable_pit", true);
    this->declare_parameter("pit_in_condition_threshold", 1000);
    this->declare_parameter("pit_stop_time_threshold", 3.0);

    goal_position_.position.x = this->get_parameter("goal.position.x").as_double();
    goal_position_.position.y = this->get_parameter("goal.position.y").as_double();
    goal_position_.position.z = this->get_parameter("goal.position.z").as_double();
    goal_position_.orientation.x = this->get_parameter("goal.orientation.x").as_double();
    goal_position_.orientation.y = this->get_parameter("goal.orientation.y").as_double();
    goal_position_.orientation.z = this->get_parameter("goal.orientation.z").as_double();
    goal_position_.orientation.w = this->get_parameter("goal.orientation.w").as_double();


    half_goal_position_.position.x = this->get_parameter("half_goal.position.x").as_double();
    half_goal_position_.position.y = this->get_parameter("half_goal.position.y").as_double();
    half_goal_position_.position.z = this->get_parameter("half_goal.position.z").as_double();
    half_goal_position_.orientation.x = this->get_parameter("half_goal.orientation.x").as_double();
    half_goal_position_.orientation.y = this->get_parameter("half_goal.orientation.y").as_double();
    half_goal_position_.orientation.z = this->get_parameter("half_goal.orientation.z").as_double();
    half_goal_position_.orientation.w = this->get_parameter("half_goal.orientation.w").as_double();

    pit_position_.position.x = this->get_parameter("pit_goal.position.x").as_double();
    pit_position_.position.y = this->get_parameter("pit_goal.position.y").as_double();
    pit_position_.position.z = this->get_parameter("pit_goal.position.z").as_double();
    pit_position_.orientation.x = this->get_parameter("pit_goal.orientation.x").as_double();
    pit_position_.orientation.y = this->get_parameter("pit_goal.orientation.y").as_double();
    pit_position_.orientation.z = this->get_parameter("pit_goal.orientation.z").as_double();
    pit_position_.orientation.w = this->get_parameter("pit_goal.orientation.w").as_double();

    goal_range_ = this->get_parameter("goal_range").as_double();
    pit_in_range_ = this->get_parameter("pit_in_range").as_double();

    enable_pit_ = this->get_parameter("enable_pit").as_bool();
    pit_in_condition_threshold_ = this->get_parameter("pit_in_condition_threshold").as_int();
    pit_stop_time_threshold_ = this->get_parameter("pit_stop_time_threshold").as_double();
}

void GoalPosePublisher::on_timer()
{
    if (++delay_count_ <= 10) {
        return;
    }

    if (!stop_initializing_pose_) {
        if (ekf_trigger_client_->service_is_ready()) {
            const auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            ekf_trigger_client_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
            {
                stop_initializing_pose_ = future.get()->success;
                delay_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Complete localization trigger");
            });
            RCLCPP_INFO(this->get_logger(), "Call localization trigger");
        }
        return;
    }

    if (!stop_streaming_goal_pose_)
    {
        auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "map";
        msg->pose.position.x = this->get_parameter("goal.position.x").as_double();
        msg->pose.position.y = this->get_parameter("goal.position.y").as_double();
        msg->pose.position.z = this->get_parameter("goal.position.z").as_double();
        msg->pose.orientation.x = this->get_parameter("goal.orientation.x").as_double();
        msg->pose.orientation.y = this->get_parameter("goal.orientation.y").as_double();
        msg->pose.orientation.z = this->get_parameter("goal.orientation.z").as_double();
        msg->pose.orientation.w = this->get_parameter("goal.orientation.w").as_double();

        goal_publisher_->publish(*msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "Publishing goal pose");
    }
}

void GoalPosePublisher::route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
    if (stop_streaming_goal_pose_ == false && msg->state >= autoware_adapi_v1_msgs::msg::RouteState::SET)
    {
        stop_streaming_goal_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Stop streaming goal pose");
    }
    is_started_ = true;
}

void GoalPosePublisher::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!is_started_)
        return;

    const auto publish_goal_pose = [this](const geometry_msgs::msg::Pose &pose) {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose = pose;
        goal_publisher_->publish(goal_pose);
    };

    const auto publish_gear_command = [this](const uint8_t &command) {
        autoware_auto_vehicle_msgs::msg::GearCommand gear_cmd;
        gear_cmd.stamp = this->get_clock()->now();
        gear_cmd.command = command;
        gear_command_publisher_->publish(gear_cmd);
    };

    switch(current_goal_state_) {

      case CurrentGoalState::NONE:
        // Publish half goal pose for start
        publish_goal_pose(half_goal_position_);
        current_goal_state_ = CurrentGoalState::HALF_GOAL;
        RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for start");
        break;

      case CurrentGoalState::HALF_GOAL:
        if(tier4_autoware_utils::calcDistance2d(msg->pose.pose, half_goal_position_) < goal_range_) {
            // Publish goal pose for loop
            publish_goal_pose(goal_position_);
            current_goal_state_ = CurrentGoalState::GOAL;
            RCLCPP_INFO(this->get_logger(), "Publishing goal pose for loop");
        }
        break;

      case CurrentGoalState::GOAL:
          if(tier4_autoware_utils::calcDistance2d(msg->pose.pose, goal_position_) < goal_range_) {
            // Publish pit_in goal pose if pit_in condition is satisfied
            if(enable_pit_ == true && (pit_condition_ >= pit_in_condition_threshold_)){
                publish_goal_pose(pit_position_);
                current_goal_state_ = CurrentGoalState::PIT_IN;
                RCLCPP_INFO(this->get_logger(), "Publishing pit_in goal pose for pit in");
            }
            // Publish half goal pose for loop
            else {
              publish_goal_pose(half_goal_position_);
              current_goal_state_ = CurrentGoalState::HALF_GOAL;
              RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for loop");
            }
          }
          break;

      case CurrentGoalState::PIT_IN:
          if(tier4_autoware_utils::calcDistance2d(msg->pose.pose, pit_position_) < pit_in_range_) {
              publish_gear_command(autoware_auto_vehicle_msgs::msg::GearCommand::PARK);
              RCLCPP_INFO(this->get_logger(), "Publishing park gear command for pit stop");

              publish_goal_pose(half_goal_position_);
              RCLCPP_INFO(this->get_logger(), "Publishing half goal pose for restart");

              current_goal_state_ = CurrentGoalState::PIT_STOP;
          }
          break;

      case CurrentGoalState::PIT_STOP:
          // Publish half goal pose if pit_stop completed for loop
          if(pit_stop_time_ >= pit_stop_time_threshold_){
              publish_gear_command(autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE);
              current_goal_state_ = CurrentGoalState::HALF_GOAL;
              RCLCPP_INFO(this->get_logger(), "Publishing drive gear command for restart");
          }
          break;
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPosePublisher>());
    rclcpp::shutdown();
    return 0;
}

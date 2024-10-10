// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "actuation_cmd_converter.hpp"

ActuationCmdConverter::ActuationCmdConverter(const rclcpp::NodeOptions & node_options)
: Node("actuation_cmd_converter", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const std::string csv_path_accel_map = declare_parameter<std::string>("csv_path_accel_map");
  const std::string csv_path_brake_map = declare_parameter<std::string>("csv_path_brake_map");
  steer_delay_sec_ = this->declare_parameter<double>("steer_delay_sec");
  delay_ = std::chrono::duration<double>(steer_delay_sec_);
  // Subscriptions
  sub_actuation_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1, std::bind(&ActuationCmdConverter::on_actuation_cmd, this, _1));
  sub_gear_ = create_subscription<GearReport>(
    "/vehicle/status/gear_status", 1, std::bind(&ActuationCmdConverter::on_gear_report, this, _1));
  sub_velocity_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1, std::bind(&ActuationCmdConverter::on_velocity_report, this, _1));

  // Publishers
  pub_ackermann_ = create_publisher<AckermannControlCommand>("/awsim/control_cmd", 1);

  // Load accel/brake map
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
    throw std::runtime_error("Cannot read accelmap.");
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
    throw std::runtime_error("Cannot read brakemap.");
  }

  delayed_cmd_pub_thread_ = std::thread([this]() {
    auto rate = rclcpp::Rate(200);

    while (rclcpp::ok()) {
      pop_and_publish_delayed_cmd(now());
      rate.sleep();
    }
  });
}

void ActuationCmdConverter::on_gear_report(const GearReport::ConstSharedPtr msg)
{
  gear_report_ = msg;
}

void ActuationCmdConverter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}

void ActuationCmdConverter::on_actuation_cmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  // Wait for input data
  if (!gear_report_ || !velocity_report_) {
    return;
  }

  const double velocity = std::abs(velocity_report_->longitudinal_velocity);
  const double acceleration = get_acceleration(*msg, velocity);

  // clamp steer cmd: -35 deg to 35 deg
  const double clamped_steer_cmd = std::clamp(msg->actuation.steer_cmd, -0.61, 0.61);

  // Publish ControlCommand
  constexpr float nan = std::numeric_limits<double>::quiet_NaN();
  AckermannControlCommand output;
  output.stamp = msg->header.stamp;
  output.lateral.steering_tire_angle = clamped_steer_cmd;
  output.lateral.steering_tire_rotation_rate = nan;
  output.longitudinal.speed = nan;
  output.longitudinal.acceleration = static_cast<float>(acceleration);

  received_cmd_queue_.emplace_back(output);

  // pub_ackermann_->publish(output);
}

double ActuationCmdConverter::get_acceleration(const ActuationCommandStamped & cmd, const double velocity)
{
  const double desired_pedal = cmd.actuation.accel_cmd - cmd.actuation.brake_cmd;
  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(+desired_pedal, velocity, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, velocity, ref_acceleration);
  }
  return ref_acceleration;
}

double ActuationCmdConverter::get_delayed_steer_cmd(const rclcpp::Time& current_time)
{
  // Delete old steer_cmd
  while (!steer_cmd_history_.empty() && 
         (current_time - steer_cmd_history_.front().first).seconds() > delay_.count())
  {
    steer_cmd_history_.pop_front();
  }
  return steer_cmd_history_.empty() ? 0.0 : steer_cmd_history_.front().second;
}

void ActuationCmdConverter::pop_and_publish_delayed_cmd(const rclcpp::Time& current_time) {
  // RCLCPP_WARN(get_logger(), "current_time: %f", current_time.seconds());

  while(!received_cmd_queue_.empty()) {
    // get the oldest command
    const auto& queue_cmd = received_cmd_queue_.front();

    // check the elapsed time from the command published
    const auto elapsed_from_cmd_published = current_time - queue_cmd.stamp;
    if(elapsed_from_cmd_published < delay_) {
      // if the elapsed time is less than the delay time, wait for the delay time
      return;
    }

    // publish and pop the command if the elapsed time is more than the delay time
    pub_ackermann_->publish(queue_cmd);
    received_cmd_queue_.pop_front();
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "Published delayed command elapsed time: %f", elapsed_from_cmd_published.seconds());
    RCLCPP_INFO(get_logger(), "Published delayed command elapsed time: %f", elapsed_from_cmd_published.seconds());
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ActuationCmdConverter)

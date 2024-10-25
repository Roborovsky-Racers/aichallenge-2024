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

  accel_limit_ = this->declare_parameter<double>("accel_limit");
  brake_limit_ = this->declare_parameter<double>("brake_limit");
  steer_limit_ = this->declare_parameter<double>("steer_limit");
  steer_rate_limit_ = this->declare_parameter<double>("steer_rate_limit");
  steer_delay_sec_ = this->declare_parameter<double>("steer_delay_sec");
  accel_delay_sec_ = this->declare_parameter<double>("accel_delay_sec");
  steering_tire_angle_gain_var_ = this->declare_parameter<double>("steering_tire_angle_gain_var");

  accel_delay_ = std::chrono::duration<double>(accel_delay_sec_);
  steer_delay_ = std::chrono::duration<double>(steer_delay_sec_);

  // Subscriptions
  sub_actuation_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1, std::bind(&ActuationCmdConverter::on_actuation_cmd, this, _1));
  sub_gear_ = create_subscription<GearReport>(
    "/vehicle/status/gear_status", 1, std::bind(&ActuationCmdConverter::on_gear_report, this, _1));
  sub_velocity_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 1, std::bind(&ActuationCmdConverter::on_velocity_report, this, _1));
  sub_steering_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1, std::bind(&ActuationCmdConverter::on_steering_report, this, _1));

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
}

void ActuationCmdConverter::on_gear_report(const GearReport::ConstSharedPtr msg)
{
  gear_report_ = msg;
}

void ActuationCmdConverter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}

void ActuationCmdConverter::on_steering_report(const SteeringReport::ConstSharedPtr msg)
{
  steering_report_ = msg;
}

void ActuationCmdConverter::on_actuation_cmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  // Wait for input data
  if (!gear_report_ || !velocity_report_ || !steering_report_) {
    return;
  }

  // use node now instead of msg->header.stamp
  const auto now = this->now();

  const auto create_ackermann_command = [&now](const double accel_cmd, const double steer_cmd) {
      constexpr float nan = std::numeric_limits<double>::quiet_NaN();
      AckermannControlCommand output;
      output.stamp = now;
      output.lateral.steering_tire_angle = steer_cmd;
      output.lateral.steering_tire_rotation_rate = nan;
      output.longitudinal.speed = nan;
      output.longitudinal.acceleration = accel_cmd;
      return output;
  };

  if(!last_ackermann_cmd_.has_value()) {
    last_ackermann_cmd_ = create_ackermann_command(0.0, 0.0);
  }

  // compute diff time between current and last ackermann command
  const rclcpp::Time last_time(last_ackermann_cmd_->stamp);
  const auto dt = (now - last_time).seconds();

  if(dt < 0.0) {
    RCLCPP_WARN(get_logger(), "detected time backward: %f -> ignore received actuation command", dt);
    return;
  } else if (dt < 1.0e-3) {
    RCLCPP_WARN(get_logger(), "detected too small dt: %f -> ignore received actuation command", dt);
    return;
  }
  if(dt > 0.5) {
    RCLCPP_WARN(get_logger(), "dt is too large: %f -> reset internal ackermann command", dt);
    last_ackermann_cmd_ = create_ackermann_command(last_ackermann_cmd_->longitudinal.acceleration, last_ackermann_cmd_->lateral.steering_tire_angle);
    return;
  }

  const double velocity = std::abs(velocity_report_->longitudinal_velocity);
  const double acceleration = get_acceleration(*msg, velocity);

  // Add accel|steer_cmd to history.
  accel_cmd_history_.emplace_back(now, acceleration);
  steer_cmd_history_.emplace_back(now, msg->actuation.steer_cmd);

  // Get delayed accel and steer command with clamp
  const double delayed_accel_cmd = std::clamp(get_delayed_cmd(accel_cmd_history_, now, accel_delay_), brake_limit_, accel_limit_);
  double delayed_steer_cmd = std::clamp(get_delayed_cmd(steer_cmd_history_, now, steer_delay_), -steer_limit_, steer_limit_);

  // limit steer command by steer rate
  const auto steer_rate = (delayed_steer_cmd - last_ackermann_cmd_->lateral.steering_tire_angle) / dt;
  if(std::abs(steer_rate) > steer_rate_limit_) {
    const double sign = steer_rate > 0.0 ? 1.0 : -1.0;
    delayed_steer_cmd = last_ackermann_cmd_->lateral.steering_tire_angle + sign * steer_rate_limit_ * dt;
  } 

  // Publish ControlCommand
  const auto output = create_ackermann_command(delayed_accel_cmd, delayed_steer_cmd);

  // 実機の挙動を模すために、指定ステアリング角度をgainで除して出力する
  // （steer_rate_limit は gainで除す前の値に対して適用されることに注意！）
  auto scaled_output = output;
  scaled_output.lateral.steering_tire_angle /= steering_tire_angle_gain_var_;
  pub_ackermann_->publish(scaled_output);

  // store last ackermann command
  last_ackermann_cmd_ = output;
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

double ActuationCmdConverter::get_delayed_cmd(
  std::deque<std::pair<rclcpp::Time, double>>& cmd_history,
  const rclcpp::Time& current_time,
  const std::chrono::duration<double>& delay)
{
  // Delete old cmd
  while (!cmd_history.empty() && 
         (current_time - cmd_history.front().first).seconds() > delay.count())
  {
    cmd_history.pop_front();
  }
  return cmd_history.empty() ? 0.0 : cmd_history.front().second;
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ActuationCmdConverter)

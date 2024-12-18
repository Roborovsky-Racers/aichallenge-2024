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

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include <deque>

#include <raw_vehicle_cmd_converter/accel_map.hpp>
#include <raw_vehicle_cmd_converter/brake_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;

class ActuationCmdConverter : public rclcpp::Node
{
public:
  explicit ActuationCmdConverter(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_ackermann_;

  void on_actuation_cmd(const ActuationCommandStamped::ConstSharedPtr msg);
  void on_gear_report(const GearReport::ConstSharedPtr msg);
  void on_velocity_report(const VelocityReport::ConstSharedPtr msg);
  void on_steering_report(const SteeringReport::ConstSharedPtr msg);

  double get_acceleration(const ActuationCommandStamped & cmd, const double velocity);
  raw_vehicle_cmd_converter::AccelMap accel_map_;
  raw_vehicle_cmd_converter::BrakeMap brake_map_;
  GearReport::ConstSharedPtr gear_report_;
  VelocityReport::ConstSharedPtr velocity_report_;
  SteeringReport::ConstSharedPtr steering_report_;

  std::deque<std::pair<rclcpp::Time, double>> accel_cmd_history_, steer_cmd_history_;
  std::optional<AckermannControlCommand> last_ackermann_cmd_;

  // delay for steering command
  std::chrono::duration<double> accel_delay_, steer_delay_; 
  double get_delayed_cmd(
    std::deque<std::pair<rclcpp::Time, double>>& cmd_history,
    const rclcpp::Time& current_time,
    const std::chrono::duration<double>& delay);
  double accel_delay_sec_, steer_delay_sec_;
  double steering_tire_angle_gain_var_;

  // limits
  double accel_limit_, brake_limit_, steer_limit_, steer_rate_limit_;
};

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

// Copyright 2024 TIER IV, Inc.
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


#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <random>
#include <deque>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;

class MultimodalDistribution {
public:
    using UniquePtr = std::unique_ptr<MultimodalDistribution>;

    MultimodalDistribution(const std::vector<double>& means, const std::vector<double>& stddevs, const std::vector<double>& weights)
        : mix_dist_(0.0, 1.0), cumulative_weights_(weights.size()) {

        // Ensure the means, stddevs, and weights vectors are the same size
        if (means.size() != stddevs.size() || means.size() != weights.size()) {
            throw std::invalid_argument("Vectors of means, stddevs, and weights must be the same size.");
        }

        // Initialize normal distributions based on means and stddevs
        for (size_t i = 0; i < means.size(); ++i) {
            distributions_.emplace_back(means[i], stddevs[i]);
        }

        // Compute cumulative weights for selecting distributions
        double total_weight = 0.0;
        for (size_t i = 0; i < weights.size(); ++i) {
            total_weight += weights[i];
            cumulative_weights_[i] = total_weight;
        }

        // Normalize cumulative weights to sum to 1
        for (double& weight : cumulative_weights_) {
            weight /= total_weight;
        }
    }

    double sample(std::mt19937& generator) {
        // Generate a random number and determine which distribution to sample from
        double rand_val = mix_dist_(generator);
        size_t index = 0;
        while (index < cumulative_weights_.size() && rand_val > cumulative_weights_[index]) {
            ++index;
        }
        return distributions_[index](generator);
    }

private:
    std::vector<std::normal_distribution<double>> distributions_; // Normal distributions for each peak
    std::uniform_real_distribution<double> mix_dist_; // Uniform distribution to select the peak
    std::vector<double> cumulative_weights_; // Cumulative weights for each distribution
};

class SensorConverter : public rclcpp::Node
{
public:
  explicit SensorConverter(const rclcpp::NodeOptions & node_options);
  ~SensorConverter();

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_outlier_gnss_pose_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_cov_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_report_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_gnss_pose_;
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_pose_cov_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_report_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_report_;

  void gnss_cov_update_and_publish_loop_();
  void on_outlier_gnss_pose(const PoseStamped::ConstSharedPtr msg);
  void on_gnss_pose(const PoseStamped::ConstSharedPtr msg);
  void on_gnss_pose_cov(const PoseWithCovarianceStamped::SharedPtr msg);
  void on_imu(const Imu::ConstSharedPtr msg);
  void on_steering_report(const SteeringReport::ConstSharedPtr msg);
  void on_velocity_report(const VelocityReport::ConstSharedPtr msg);

  std::optional<PoseStamped> outlier_gnss_pose_;
  PoseStamped::SharedPtr pose_;
  PoseWithCovarianceStamped::SharedPtr pose_cov_;
  Imu::SharedPtr imu_;
  SteeringReport::SharedPtr steering_report_;
  VelocityReport::SharedPtr velocity_report_;
  int gnss_pose_delay_;
  std::atomic<double> base_yaw_;

  std::mutex pose_cov_queue_mutex_;
  std::deque<PoseWithCovarianceStamped> pose_cov_queue_;
  std::thread gnss_cov_pub_thread_;

  std::mt19937 generator_;
  std::normal_distribution<double> pose_distribution_;
  // std::normal_distribution<double> pose_cov_distribution_;
  MultimodalDistribution::UniquePtr pose_cov_distribution_x_;
  MultimodalDistribution::UniquePtr pose_cov_distribution_y_;
  std::normal_distribution<double> imu_acc_distribution_;
  std::normal_distribution<double> imu_ang_distribution_;
  std::normal_distribution<double> imu_ori_distribution_;
  std::normal_distribution<double> steering_angle_distribution_;

  double gnss_lost_probability_;
  std::bernoulli_distribution gnss_lost_dist_;
  double gnss_lost_time_mean_;
  double gnss_lost_time_stddev_;
  std::normal_distribution<double> gnss_lost_time_dist_;

  double gnss_pose_cov_publish_period_;
  double gnss_pose_cov_publish_period_mean_;
  double gnss_pose_cov_publish_period_stddev_;
  std::normal_distribution<double> gnss_pose_cov_publish_period_distribution_;

  double gnss_pose_cov_value_uptate_period_;
  double gnss_pose_cov_value_uptate_period_mean_;
  double gnss_pose_cov_value_uptate_period_stddev_;
  std::normal_distribution<double> gnss_pose_cov_value_update_period_distribution_;

  double gnss_pose_cov_delay_;
  double gnss_pose_cov_delay_mean_;
  double gnss_pose_cov_delay_stddev_;
  std::normal_distribution<double> gnss_pose_cov_delay_distribution_;

  double gnss_pose_mean_;
  double gnss_pose_stddev_;

  bool set_fake_gnss_pose_cov_;
  double gnss_pose_cov_mean_;
  double gnss_pose_cov_stddev_;

  double imu_acc_mean_;
  double imu_acc_stddev_;
  double imu_ang_mean_;
  double imu_ang_stddev_;
  double imu_ori_mean_;
  double imu_ori_stddev_;
  double steering_angle_mean_;
  double steering_angle_stddev_;
  double steering_tire_angle_gain_var_;

  bool use_sim_time_;
};

#endif // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

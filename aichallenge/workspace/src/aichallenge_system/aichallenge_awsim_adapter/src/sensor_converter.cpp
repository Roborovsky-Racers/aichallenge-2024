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

#include "sensor_converter.hpp"
#include <chrono>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <limits>
#include <thread>

// namespace {
// void normalize_quaternion(geometry_msgs::msg::Quaternion & q)
// {
//   const double norm = sqrt(
//     q.x * q.x +
//     q.y * q.y +
//     q.z * q.z +
//     q.w * q.w);

//   q.x /= norm;
//   q.y /= norm;
//   q.z /= norm;
//   q.w /= norm;
// }
// }

SensorConverter::SensorConverter(const rclcpp::NodeOptions & node_options)
: Node("sensor_converter", node_options)
{
  using std::placeholders::_1;

  // Parameters
  gnss_pose_cov_value_uptate_period_ = declare_parameter<double>("gnss_pose_cov_value_uptate_period");
  gnss_pose_cov_value_uptate_period_mean_ = declare_parameter<double>("gnss_pose_cov_value_uptate_period_mean");
  gnss_pose_cov_value_uptate_period_stddev_ = declare_parameter<double>("gnss_pose_cov_value_uptate_period_stddev");

  gnss_pose_cov_delay_ = declare_parameter<double>("gnss_pose_cov_delay");
  gnss_pose_cov_delay_mean_ = declare_parameter<double>("gnss_pose_cov_delay_mean");
  gnss_pose_cov_delay_stddev_ = declare_parameter<double>("gnss_pose_cov_delay_stddev");

  gnss_pose_cov_publish_period_ = declare_parameter<double>("gnss_pose_cov_publish_period");
  gnss_pose_cov_publish_period_mean_ = declare_parameter<double>("gnss_pose_cov_publish_period_mean");
  gnss_pose_cov_publish_period_stddev_ = declare_parameter<double>("gnss_pose_cov_publish_period_stddev");

  gnss_pose_delay_ = declare_parameter<int>("gnss_pose_delay");
  gnss_pose_mean_ = declare_parameter<double>("gnss_pose_mean");
  gnss_pose_stddev_ = declare_parameter<double>("gnss_pose_stddev");
  gnss_pose_cov_mean_ = declare_parameter<double>("gnss_pose_cov_mean");
  gnss_pose_cov_stddev_ = declare_parameter<double>("gnss_pose_cov_stddev");
  imu_acc_mean_ = declare_parameter<double>("imu_acc_mean");
  imu_acc_stddev_ = declare_parameter<double>("imu_acc_stddev");
  imu_ang_mean_ = declare_parameter<double>("imu_ang_mean");
  imu_ang_stddev_ = declare_parameter<double>("imu_ang_stddev");
  imu_ori_mean_ = declare_parameter<double>("imu_ori_mean");
  imu_ori_stddev_ = declare_parameter<double>("imu_ori_stddev");
  steering_angle_mean_ = declare_parameter<double>("steering_angle_mean");
  steering_angle_stddev_ = declare_parameter<double>("steering_angle_stddev");
  steering_tire_angle_gain_var_ = declare_parameter<double>("steering_tire_angle_gain_var");


  // Subscriptions
  sub_outlier_gnss_pose_ = create_subscription<PoseStamped>(
    "~/outlier_gnss_pose", 1, std::bind(&SensorConverter::on_outlier_gnss_pose, this, _1));
  sub_gnss_pose_ = create_subscription<PoseStamped>(
    "/awsim/gnss/pose", 1, std::bind(&SensorConverter::on_gnss_pose, this, _1));
  sub_gnss_pose_cov_ = create_subscription<PoseWithCovarianceStamped>(
    "/awsim/gnss/pose_with_covariance", 1, std::bind(&SensorConverter::on_gnss_pose_cov, this, _1));
  sub_imu_ = create_subscription<Imu>(
    "/awsim/imu", 1, std::bind(&SensorConverter::on_imu, this, _1));
  sub_steering_report_ = create_subscription<SteeringReport>(
    "/awsim/steering_status", 1, std::bind(&SensorConverter::on_steering_report, this, _1));

  // Publishers
  pub_gnss_pose_ = create_publisher<PoseStamped>("/sensing/gnss/pose", 1);
  pub_gnss_pose_cov_ = create_publisher<PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", 1);
  pub_imu_ = create_publisher<Imu>("/sensing/imu/imu_raw", 1);
  pub_steering_report_ = create_publisher<SteeringReport>("/vehicle/status/steering_status", 1);

  std::random_device rd;
  generator_ = std::mt19937(rd());
  pose_distribution_ = std::normal_distribution<double>(gnss_pose_mean_, gnss_pose_stddev_);
  pose_cov_distribution_ = std::normal_distribution<double>(gnss_pose_cov_mean_, gnss_pose_cov_stddev_);
  imu_acc_distribution_ = std::normal_distribution<double>(imu_acc_mean_, imu_acc_stddev_);
  imu_ang_distribution_ = std::normal_distribution<double>(imu_ang_mean_, imu_ang_stddev_);
  imu_ori_distribution_ = std::normal_distribution<double>(imu_ori_mean_, imu_ori_stddev_);
  steering_angle_distribution_ = std::normal_distribution<double>(steering_angle_mean_, steering_angle_stddev_);

  gnss_pose_cov_value_update_period_distribution_ = std::normal_distribution<double>(gnss_pose_cov_value_uptate_period_mean_, gnss_pose_cov_value_uptate_period_stddev_);
  gnss_pose_cov_delay_distribution_ = std::normal_distribution<double>(gnss_pose_cov_delay_mean_, gnss_pose_cov_delay_stddev_);
  gnss_pose_cov_publish_period_distribution_ = std::normal_distribution<double>(gnss_pose_cov_publish_period_mean_, gnss_pose_cov_publish_period_stddev_);

  gnss_cov_pub_thread_ = std::thread(std::bind(&SensorConverter::gnss_cov_update_and_publish_loop_, this));
}

SensorConverter::~SensorConverter() {
  if (gnss_cov_pub_thread_.joinable()) {
    gnss_cov_pub_thread_.join();
  }
}

void SensorConverter::on_outlier_gnss_pose(const PoseStamped::ConstSharedPtr msg) {
  outlier_gnss_pose_ = *msg;
}

void SensorConverter::gnss_cov_update_and_publish_loop_() {

  const auto get_next_gnss_pose_cov_value_uptate_period = [this]() {
    const auto v = std::max(
      gnss_pose_cov_value_uptate_period_,
      gnss_pose_cov_value_uptate_period_ + gnss_pose_cov_value_update_period_distribution_(generator_));
    RCLCPP_DEBUG(get_logger(), "Next GNSS value update period: %f", v);
    return v;
  };

  const auto get_next_gnss_pose_cov_delay = [this]() {
    const auto v = std::max(
      gnss_pose_cov_delay_,
      gnss_pose_cov_delay_ + gnss_pose_cov_delay_distribution_(generator_));
    RCLCPP_DEBUG(get_logger(), "Next GNSS pose cov delay: %f", v);
    return v;
  };

  const auto get_next_gnss_pose_cov_publish_period = [this]() {
    const auto v = std::max(
      gnss_pose_cov_publish_period_,
      gnss_pose_cov_publish_period_ + gnss_pose_cov_publish_period_distribution_(generator_));
    // RCLCPP_DEBUG(get_logger(), "Next GNSS pose cov publish period: %f", v);
    return v;
  };

  const auto pop_nearest_gnss_cov = [this](const auto& stamp) -> std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> {
      std::lock_guard<std::mutex> lock(pose_cov_queue_mutex_);

      if (pose_cov_queue_.empty()) {
          return std::nullopt;
      }

      auto nearest_gnss_cov = pose_cov_queue_.front();
      pose_cov_queue_.pop_front();

      double min_diff_time = std::abs((stamp - nearest_gnss_cov.header.stamp).seconds());

      while (!pose_cov_queue_.empty()) {
          const auto current_cov = pose_cov_queue_.front();
          const double diff_time = std::abs((stamp - current_cov.header.stamp).seconds());

          if (diff_time > min_diff_time) {
              break;
          }

          nearest_gnss_cov = current_cov;
          min_diff_time = diff_time;
          pose_cov_queue_.pop_front();
      }

      return nearest_gnss_cov;
  };

  const auto publish_gnss_cov_with_now_stamp = [this](const auto& gnss_cov_msg) {
      auto msg = gnss_cov_msg;
      msg.header.stamp = get_clock()->now();
      pub_gnss_pose_cov_->publish(msg);
  };

  const auto to_nanoseconds = [](double seconds) -> std::chrono::nanoseconds  {
    auto duration_in_seconds = std::chrono::duration<double>(seconds);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_in_seconds);
  };

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_gnss_pose_cov;
  double next_gnss_value_uptate_period = get_next_gnss_pose_cov_value_uptate_period();
  double next_gnss_pose_cov_delay = get_next_gnss_pose_cov_delay();

  auto prev_time = get_clock()->now();

  while(rclcpp::ok()) {
    // Wait until the next publish period
#if 1
    rclcpp::WallRate(1.0 / get_next_gnss_pose_cov_publish_period()).sleep();
#else
    {
      const auto loop_start = get_clock()->now();
      const auto next_wakeup_time = prev_time + to_nanoseconds(get_next_gnss_pose_cov_publish_period());
      const auto sleep_time = (next_wakeup_time - loop_start).to_chrono<std::chrono::nanoseconds>();
      if (sleep_time > std::chrono::nanoseconds(0)) {
        RCLCPP_WARN(get_logger(), "Sleep for %f", sleep_time.count() / 1e9);
        rclcpp::sleep_for(sleep_time);
      }
      prev_time = loop_start;
    }
#endif

    const auto now = get_clock()->now();

    // If the pose is older than the value update period, publish the last updated pose
    if(last_gnss_pose_cov.has_value() &&
      (now - last_gnss_pose_cov->header.stamp).seconds() < next_gnss_value_uptate_period + next_gnss_pose_cov_delay) {
      publish_gnss_cov_with_now_stamp(*last_gnss_pose_cov);
      continue;
    }

    // Calculate the timestamp to get past data considering the delay of GNSS
    next_gnss_pose_cov_delay = get_next_gnss_pose_cov_delay();
    const auto past_stamp_with_delay = now - to_nanoseconds(next_gnss_pose_cov_delay);

    // Try to get the nearest past GNSS pose covariance
    if(const auto nearest_past_pose_cov = pop_nearest_gnss_cov(past_stamp_with_delay)) {
      // If the nearest pose got, update and publish the pose
      last_gnss_pose_cov = *nearest_past_pose_cov;
      publish_gnss_cov_with_now_stamp(*last_gnss_pose_cov);

      // Update the next value update period
      next_gnss_value_uptate_period = get_next_gnss_pose_cov_value_uptate_period();
    }
  }
}

void SensorConverter::on_gnss_pose(const PoseStamped::ConstSharedPtr msg)
{
  auto process_and_publish_gnss = [this, msg]() {
    rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_delay_));
    
    auto pose = std::make_shared<PoseStamped>(*msg);
    pose->header.stamp = now();
    pose->pose.position.x += pose_distribution_(generator_);
    pose->pose.position.y += pose_distribution_(generator_);
    pose->pose.position.z += pose_distribution_(generator_);
    pose->pose.orientation.x += pose_distribution_(generator_);
    pose->pose.orientation.y += pose_distribution_(generator_);
    pose->pose.orientation.z += pose_distribution_(generator_);
    pose->pose.orientation.w += pose_distribution_(generator_);

    pub_gnss_pose_->publish(*pose);
  };

  std::thread processing_thread(process_and_publish_gnss);
  processing_thread.detach();
}

void SensorConverter::on_gnss_pose_cov(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
#if 1
  std::lock_guard<std::mutex> lock(pose_cov_queue_mutex_);
  pose_cov_queue_.push_back(*msg);
#elif 0
  const auto process_gnss_cov = [this, msg]() {
    auto noised_pose_cov = std::make_shared<PoseWithCovarianceStamped>(*msg);
    noised_pose_cov->header.stamp = now();
    noised_pose_cov->pose.pose.position.x += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.position.y += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.position.z += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.orientation.x += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.orientation.y += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.orientation.z += pose_cov_distribution_(generator_);
    noised_pose_cov->pose.pose.orientation.w += pose_cov_distribution_(generator_);
    // normalize_quaternion(noised_pose_cov->pose.pose.orientation);

    if(outlier_gnss_pose_.has_value()) {
      noised_pose_cov->pose.pose.position.x += outlier_gnss_pose_->pose.position.x;
      noised_pose_cov->pose.pose.position.y += outlier_gnss_pose_->pose.position.y;
      outlier_gnss_pose_ = std::nullopt;
      // RCLCPP_WARN(get_logger(), "Outlier GNSS pose detected! x: %f, y: %f",
      //             outlier_gnss_pose_->pose.position.x, outlier_gnss_pose_->pose.position.y);
    }

    return noised_pose_cov;
  };

  // If the pose is older than the value update period, update the pose
  if (pose_cov_ == nullptr || (now() - pose_cov_->header.stamp).seconds() >= next_gnss_value_uptate_period_) {
    pose_cov_ = process_gnss_cov();
    next_gnss_value_uptate_period_ = std::max(gnss_value_uptate_period_, gnss_value_uptate_period_ + gnss_update_period_distribution_(generator_));

    next_gnss_pose_cov_delay_  = std::max(gnss_pose_cov_delay_, gnss_pose_cov_delay_ + static_cast<int>(gnss_pose_cov_delay_distribution_(generator_) * 1000));

    int next_gnss_value_uptate_period_msec = static_cast<int>(next_gnss_value_uptate_period_ * 1000.0 + 0.05);

    if(next_gnss_pose_cov_delay_ < next_gnss_value_uptate_period_msec + 50) {
      next_gnss_pose_cov_delay_ = next_gnss_value_uptate_period_msec + 50;
    }

    // RCLCPP_WARN(get_logger(), "Next GNSS value update period: %f, delay: %d", next_gnss_value_uptate_period_, next_gnss_pose_cov_delay_);
  }

  const auto delayed_publish = [this](PoseWithCovarianceStamped pose_cov) {
    rclcpp::sleep_for(std::chrono::milliseconds(next_gnss_pose_cov_delay_));
    pose_cov.header.stamp = now();
    pub_gnss_pose_cov_->publish(pose_cov);
  };

  std::thread delayed_publish_thread(std::bind(delayed_publish, *pose_cov_));
  delayed_publish_thread.detach();
#else
    auto process_and_publish_gnss_cov = [this, msg]() {
    rclcpp::sleep_for(std::chrono::milliseconds(gnss_pose_delay_));
    
    auto pose_cov = std::make_shared<PoseWithCovarianceStamped>(*msg);
    pose_cov->header.stamp = now();
    pose_cov->pose.pose.position.x += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.position.y += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.position.z += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.x += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.y += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.z += pose_cov_distribution_(generator_);
    pose_cov->pose.pose.orientation.w += pose_cov_distribution_(generator_);

    pub_gnss_pose_cov_->publish(*pose_cov);
  };

  std::thread processing_thread(process_and_publish_gnss_cov);
  processing_thread.detach();
#endif
}

void SensorConverter::on_imu(const Imu::ConstSharedPtr msg)
{
  imu_ = std::make_shared<Imu>(*msg);
  imu_ -> orientation.x += imu_ori_distribution_(generator_);
  imu_ -> orientation.y += imu_ori_distribution_(generator_);
  imu_ -> orientation.z += imu_ori_distribution_(generator_);
  imu_ -> orientation.w += imu_ori_distribution_(generator_);
  imu_ -> angular_velocity.x += imu_ang_distribution_(generator_);
  imu_ -> angular_velocity.y += imu_ang_distribution_(generator_);
  imu_ -> angular_velocity.z += imu_ang_distribution_(generator_);
  imu_ -> linear_acceleration.x += imu_acc_distribution_(generator_);
  imu_ -> linear_acceleration.y += imu_acc_distribution_(generator_);
  imu_ -> linear_acceleration.z += imu_acc_distribution_(generator_);
  pub_imu_->publish(*imu_);
}

void SensorConverter::on_steering_report(const SteeringReport::ConstSharedPtr msg)
{
  steering_report_ = std::make_shared<SteeringReport>(*msg);

  // 実機の挙動に合わせて、実際のステア角に対してgainを掛けた値を steer status として出力する
  steering_report_->steering_tire_angle *= steering_tire_angle_gain_var_;

  steering_report_->steering_tire_angle += steering_angle_distribution_(generator_);
  pub_steering_report_->publish(*steering_report_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SensorConverter)

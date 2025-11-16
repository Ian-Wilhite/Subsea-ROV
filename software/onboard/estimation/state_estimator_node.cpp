#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double wrapAngle(double angle)
{
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

bool isFiniteVector(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}  // namespace

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : rclcpp::Node("state_estimator"), orientation_(0.0, 0.0, 0.0, 1.0)
  {
    const auto odom_topic =
      this->declare_parameter<std::string>("odom_topic", "/obc/state/odometry");
    const auto imu_topic =
      this->declare_parameter<std::string>("imu_topic", "/obc/imu/data");
    const auto mag_topic =
      this->declare_parameter<std::string>("mag_topic", "/obc/imu/mag");
    const auto depth_topic =
      this->declare_parameter<std::string>("depth_topic", "/obc/depth/raw");
    const auto flow_topic =
      this->declare_parameter<std::string>("optical_flow_topic", "");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    yaw_gain_ = this->declare_parameter<double>("magnetometer_yaw_gain", 0.1);
    depth_alpha_ = this->declare_parameter<double>("depth_alpha", 1.0);
    flow_gain_ = this->declare_parameter<double>("flow_gain", 0.8);
    gravity_ = this->declare_parameter<double>("gravity", 9.80665);
    depth_positive_down_ = this->declare_parameter<bool>("depth_positive_down", true);
    mag_timeout_sec_ = this->declare_parameter<double>("mag_timeout", 0.5);
    depth_timeout_sec_ = this->declare_parameter<double>("depth_timeout", 1.0);
    flow_timeout_sec_ = this->declare_parameter<double>("flow_timeout", 0.3);

    auto sensor_qos = rclcpp::SensorDataQoS();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1));
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic, sensor_qos,
      std::bind(&StateEstimatorNode::magnetometerCallback, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      depth_topic, sensor_qos,
      std::bind(&StateEstimatorNode::depthCallback, this, std::placeholders::_1));
    if (!flow_topic.empty()) {
      flow_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        flow_topic, sensor_qos,
        std::bind(&StateEstimatorNode::flowCallback, this, std::placeholders::_1));
    }

    odom_msg_.pose.covariance.fill(0.0);
    odom_msg_.twist.covariance.fill(0.0);
    odom_msg_.pose.covariance[0] = 0.05;
    odom_msg_.pose.covariance[7] = 0.05;
    odom_msg_.pose.covariance[14] = 0.02;
    odom_msg_.pose.covariance[21] = 0.01;
    odom_msg_.pose.covariance[28] = 0.01;
    odom_msg_.pose.covariance[35] = 0.05;
    odom_msg_.twist.covariance[0] = 0.04;
    odom_msg_.twist.covariance[7] = 0.04;
    odom_msg_.twist.covariance[14] = 0.04;
    odom_msg_.twist.covariance[21] = 0.02;
    odom_msg_.twist.covariance[28] = 0.02;
    odom_msg_.twist.covariance[35] = 0.02;
  }

private:
  rclcpp::Time sanitizeStamp(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return this->get_clock()->now();
    }
    return rclcpp::Time(stamp);
  }

  bool magnetometerFresh(const rclcpp::Time & stamp) const
  {
    if (!have_mag_) {
      return false;
    }
    if (mag_timeout_sec_ <= 0.0) {
      return true;
    }
    const double age = (stamp - last_mag_stamp_).seconds();
    return age >= 0.0 && age <= mag_timeout_sec_;
  }

  bool depthFresh(const rclcpp::Time & stamp) const
  {
    if (!have_depth_) {
      return false;
    }
    if (depth_timeout_sec_ <= 0.0) {
      return true;
    }
    const double age = (stamp - last_depth_stamp_).seconds();
    return age >= 0.0 && age <= depth_timeout_sec_;
  }

  bool flowFresh(const rclcpp::Time & stamp) const
  {
    if (!have_flow_) {
      return false;
    }
    if (flow_timeout_sec_ <= 0.0) {
      return true;
    }
    const double age = (stamp - last_flow_stamp_).seconds();
    return age >= 0.0 && age <= flow_timeout_sec_;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg == nullptr) {
      return;
    }
    if (!isFiniteVector(msg->linear_acceleration) || !isFiniteVector(msg->angular_velocity)) {
      return;
    }

    const rclcpp::Time stamp = sanitizeStamp(msg->header.stamp);
    tf2::Quaternion imu_q(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    if (imu_q.length2() < 1e-12) {
      imu_q.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      imu_q.normalize();
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(imu_q).getRPY(roll, pitch, yaw);
    if (magnetometerFresh(stamp)) {
      const double yaw_mag = std::atan2(magnetometer_[1], magnetometer_[0]);
      const double yaw_error = wrapAngle(yaw_mag - yaw);
      yaw = wrapAngle(yaw + yaw_gain_ * yaw_error);
    }
    orientation_.setRPY(roll, pitch, yaw);
    orientation_.normalize();

    double dt = imu_initialized_ ? (stamp - last_imu_stamp_).seconds() : 0.0;
    if (dt < 0.0) {
      dt = 0.0;
    } else if (dt > 0.2) {
      dt = 0.2;
    }
    last_imu_stamp_ = stamp;
    if (!imu_initialized_) {
      imu_initialized_ = true;
    }

    const tf2::Matrix3x3 rotation(orientation_);
    tf2::Vector3 accel_body(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    tf2::Vector3 accel_world = rotation * accel_body;
    accel_world.setZ(accel_world.z() - gravity_);

    velocity_[0] += accel_world.x() * dt;
    velocity_[1] += accel_world.y() * dt;
    velocity_[2] += accel_world.z() * dt;

    if (flowFresh(stamp)) {
      velocity_[0] += flow_gain_ * (flow_linear_[0] - velocity_[0]);
      velocity_[1] += flow_gain_ * (flow_linear_[1] - velocity_[1]);
      velocity_[2] += flow_gain_ * (flow_linear_[2] - velocity_[2]);
    }

    position_[0] += velocity_[0] * dt;
    position_[1] += velocity_[1] * dt;
    position_[2] += velocity_[2] * dt;
    if (depthFresh(stamp)) {
      const double desired_z = depth_positive_down_ ? -depth_value_ : depth_value_;
      position_[2] += depth_alpha_ * (desired_z - position_[2]);
      velocity_[2] = 0.0;
    }

    publishOdometry(stamp, *msg);
  }

  void magnetometerCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    if (msg == nullptr) {
      return;
    }
    if (!isFiniteVector(msg->magnetic_field)) {
      return;
    }
    magnetometer_[0] = msg->magnetic_field.x;
    magnetometer_[1] = msg->magnetic_field.y;
    magnetometer_[2] = msg->magnetic_field.z;
    last_mag_stamp_ = sanitizeStamp(msg->header.stamp);
    have_mag_ = true;
  }

  void depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg == nullptr || !std::isfinite(msg->data)) {
      return;
    }
    depth_value_ = msg->data;
    last_depth_stamp_ = this->get_clock()->now();
    have_depth_ = true;
  }

  void flowCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    if (msg == nullptr || !isFiniteVector(msg->twist.twist.linear)) {
      return;
    }
    flow_linear_[0] = msg->twist.twist.linear.x;
    flow_linear_[1] = msg->twist.twist.linear.y;
    flow_linear_[2] = msg->twist.twist.linear.z;
    last_flow_stamp_ = sanitizeStamp(msg->header.stamp);
    have_flow_ = true;
  }

  void publishOdometry(const rclcpp::Time & stamp, const sensor_msgs::msg::Imu & imu_msg)
  {
    odom_msg_.header.stamp = stamp;
    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id = base_frame_;
    odom_msg_.pose.pose.position.x = position_[0];
    odom_msg_.pose.pose.position.y = position_[1];
    odom_msg_.pose.pose.position.z = position_[2];
    odom_msg_.pose.pose.orientation = tf2::toMsg(orientation_);
    odom_msg_.twist.twist.linear.x = velocity_[0];
    odom_msg_.twist.twist.linear.y = velocity_[1];
    odom_msg_.twist.twist.linear.z = velocity_[2];
    odom_msg_.twist.twist.angular = imu_msg.angular_velocity;
    odom_pub_->publish(odom_msg_);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr flow_sub_;

  nav_msgs::msg::Odometry odom_msg_;
  std::array<double, 3> position_{};
  std::array<double, 3> velocity_{};
  tf2::Quaternion orientation_;
  bool imu_initialized_{false};
  rclcpp::Time last_imu_stamp_;

  std::array<double, 3> magnetometer_{};
  rclcpp::Time last_mag_stamp_;
  bool have_mag_{false};

  double depth_value_{0.0};
  rclcpp::Time last_depth_stamp_;
  bool have_depth_{false};

  std::array<double, 3> flow_linear_{};
  rclcpp::Time last_flow_stamp_;
  bool have_flow_{false};

  double yaw_gain_{0.1};
  double depth_alpha_{1.0};
  double flow_gain_{0.8};
  bool depth_positive_down_{true};
  double gravity_{9.80665};
  double mag_timeout_sec_{0.5};
  double depth_timeout_sec_{1.0};
  double flow_timeout_sec_{0.3};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}

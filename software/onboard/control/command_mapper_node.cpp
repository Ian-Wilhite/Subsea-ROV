#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class CommandMapperNode : public rclcpp::Node
{
public:
  explicit CommandMapperNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("command_mapper", options)
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/gcs/cmd_vel");
    state_est_topic_ = declare_parameter<std::string>("state_est_topic", "/obc/state/est");

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, reliable_qos,
      std::bind(&CommandMapperNode::HandleCmdVel, this, std::placeholders::_1));

    state_est_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      state_est_topic_, reliable_qos,
      std::bind(&CommandMapperNode::HandleStateEst, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to %s and %s", cmd_vel_topic_.c_str(), state_est_topic_.c_str());
  }

private:
  void HandleCmdVel(const geometry_msgs::msg::Twist::SharedPtr & msg)
  {
    (void)msg;
  }

  void HandleStateEst(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    (void)msg;
  }

  std::string cmd_vel_topic_;
  std::string state_est_topic_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_est_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options);

  auto node = std::make_shared<CommandMapperNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// SPDX-FileCopyrightText: 2024 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "teleop_twist_joy_comfy/teleop_twist_joy_comfy.hpp"

#include <algorithm>
#include <memory>

namespace teleop_twist_joy_comfy
{

TeleopTwistJoyComfy::TeleopTwistJoyComfy(const rclcpp::NodeOptions & options)
: Node("teleop_twist_joy_comfy", options)
{
  getParam();

  initPublisher();
  initSubscription();
}

void TeleopTwistJoyComfy::getParam()
{
  linear_x_ = this->declare_parameter("linear_x", 0.5);
  linear_x_max_ = this->declare_parameter("linear_x_max", 1.0);
  angular_z_ = this->declare_parameter("angular_z", 0.5);
  angular_z_max_ = this->declare_parameter("angular_z_max", 1.0);

  speed_up_down_linear_x_original_ = this->declare_parameter("speed_up_down_linear_x", 0.1);
  speed_up_down_angular_z_original_ = this->declare_parameter("speed_up_down_angular_z", 0.1);
  speed_up_down_scale_linear_x_ = this->declare_parameter("speed_up_down_scale_linear_x", 1.0);
  speed_up_down_scale_angular_z_ = this->declare_parameter("speed_up_down_scale_angular_z", 1.0);

  publish_twist_stamped_ = this->declare_parameter("publish_twist_stamped", true);
}

void TeleopTwistJoyComfy::initPublisher()
{
  if (!publish_twist_stamped_)
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("botwheel_explorer/cmd_vel", 10);
  else
    twist_stamped_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("botwheel_explorer/cmd_vel", 10);
}

void TeleopTwistJoyComfy::initSubscription()
{
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(),
    std::bind(&TeleopTwistJoyComfy::joy_callback, this, std::placeholders::_1));
}

void TeleopTwistJoyComfy::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons[A]) {
    changeAngularVelocity(joy);
  }

  if (joy->buttons[B] && !auto_teleop_mode_ && !button_b_state_) {
    autoTeleopStart();
  } else if (joy->buttons[B] && auto_teleop_mode_ && !button_b_state_) {
    autoTeleopEnd();
  }

  if (!joy->buttons[B] && auto_teleop_mode_) {
    autoTeleop(joy);
  }

  // if (joy->buttons[X]) {
  //   manualTeleop(joy);
  // }

  if (joy->buttons[Y]) {
    changeLinearVelocity(joy);
  }

  if (!joy->buttons[B] && !joy->buttons[X] && !auto_teleop_mode_) {
    stopVelocity();
  }

  button_b_state_ = joy->buttons[B];

  button_lb_state_ = joy->buttons[4];
  button_rb_state_ = joy->buttons[5];
}

void TeleopTwistJoyComfy::autoTeleopStart() { auto_teleop_mode_ = true; }

void TeleopTwistJoyComfy::autoTeleopEnd()
{
  auto_teleop_mode_ = false;

  stopVelocity();
}

void TeleopTwistJoyComfy::autoTeleop(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (!publish_twist_stamped_)
    publishTwist(calcTwist<geometry_msgs::msg::Twist>(joy));
  else
    publishTwist(calcTwist<geometry_msgs::msg::TwistStamped>(joy));
}

template <typename T>
std::unique_ptr<T> TeleopTwistJoyComfy::calcTwist(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  auto twist = std::make_unique<T>();

  double linear_x = 0, angular_z = 0;
  if (auto_teleop_mode_) {
    linear_x += linear_x_ + speed_up_down_linear_x_ * speed_up_down_scale_linear_x_;
    angular_z +=
      (angular_z_ + speed_up_down_angular_z_ * speed_up_down_scale_angular_z_) * joy->axes[3];

    if (linear_x < 0)
      speed_down_linear_x_lock_ = true;
    else
      speed_down_linear_x_lock_ = false;

    if (linear_x > linear_x_max_)
      speed_up_linear_x_lock_ = true;
    else
      speed_up_linear_x_lock_ = false;

    if (angular_z_ + speed_up_down_angular_z_ * speed_up_down_scale_angular_z_ < 0)
      speed_down_angular_z_lock_ = true;
    else
      speed_down_angular_z_lock_ = false;

    if (angular_z_ + speed_up_down_angular_z_ * speed_up_down_scale_angular_z_ > angular_z_max_)
      speed_up_angular_z_lock_ = true;
    else
      speed_up_angular_z_lock_ = false;

    linear_x = std::clamp(linear_x, 0., linear_x_max_);
    if (joy->axes[3] > 0)
      angular_z = std::clamp(angular_z, 0., angular_z_max_);
    else if (joy->axes[3] < 0)
      angular_z = std::clamp(angular_z, -1. * angular_z_max_, 0.);
  } else {
  }

  if constexpr (std::is_same<T, geometry_msgs::msg::Twist>::value) {
    twist->linear.x = linear_x;
    twist->angular.z = angular_z;
  } else if constexpr (std::is_same<T, geometry_msgs::msg::TwistStamped>::value) {
    twist->header.frame_id = "";
    twist->header.stamp = this->get_clock()->now();
    twist->twist.linear.x = linear_x;
    twist->twist.angular.z = angular_z;
  }

  return twist;
}

template <typename T>
void TeleopTwistJoyComfy::publishTwist(std::unique_ptr<T> twist)
{
  if constexpr (std::is_same<T, geometry_msgs::msg::Twist>::value) {
    twist_pub_->publish(std::move(twist));
  } else if constexpr (std::is_same<T, geometry_msgs::msg::TwistStamped>::value) {
    twist_stamped_pub_->publish(std::move(twist));
  }
}

template <typename T>
void TeleopTwistJoyComfy::publishZeroTwist()
{
  auto twist = std::make_unique<T>();

  if constexpr (std::is_same<T, geometry_msgs::msg::Twist>::value) {
    twist_pub_->publish(std::move(twist));
  } else if constexpr (std::is_same<T, geometry_msgs::msg::TwistStamped>::value) {
    twist->header.frame_id = "";
    twist->header.stamp = this->get_clock()->now();
    twist_stamped_pub_->publish(std::move(twist));
  }
}

void TeleopTwistJoyComfy::manualTeleop(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (!publish_twist_stamped_)
    publishTwist(calcTwist<geometry_msgs::msg::Twist>(joy));
  else
    publishTwist(calcTwist<geometry_msgs::msg::TwistStamped>(joy));
}

void TeleopTwistJoyComfy::changeLinearVelocity(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (!speed_down_linear_x_lock_) {
    if (joy->buttons[4] && !button_lb_state_)
      speed_up_down_linear_x_ -= speed_up_down_linear_x_original_;
  }

  if (!speed_up_linear_x_lock_) {
    if (joy->buttons[5] && !button_rb_state_)
      speed_up_down_linear_x_ += speed_up_down_linear_x_original_;
  }
}

void TeleopTwistJoyComfy::changeAngularVelocity(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (!speed_down_angular_z_lock_) {
    if (joy->buttons[4] && !button_lb_state_)
      speed_up_down_angular_z_ -= speed_up_down_angular_z_original_;
  }

  if (!speed_up_angular_z_lock_) {
    if (joy->buttons[5] && !button_rb_state_)
      speed_up_down_angular_z_ += speed_up_down_angular_z_original_;
  }
}

void TeleopTwistJoyComfy::stopVelocity()
{
  if (!publish_twist_stamped_)
    publishZeroTwist<geometry_msgs::msg::Twist>();
  else
    publishZeroTwist<geometry_msgs::msg::TwistStamped>();
}

}  // namespace teleop_twist_joy_comfy

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy_comfy::TeleopTwistJoyComfy)
// SPDX-FileCopyrightText: 2024 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef TELEOP_TWIST_JOY_COMFY__TELEOP_TWIST_JOY_COMFY_HPP_
#define TELEOP_TWIST_JOY_COMFY__TELEOP_TWIST_JOY_COMFY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace teleop_twist_joy_comfy
{

class TeleopTwistJoyComfy : public rclcpp::Node
{
public:
  explicit TeleopTwistJoyComfy(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initPublisher();
  void initSubscription();

  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  void autoTeleopStart();
  void autoTeleopEnd();
  void autoTeleop(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
  void manualTeleop(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  void changeLinearVelocity(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
  void changeAngularVelocity(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  void stopVelocity();

  void resetAllSpeedUpDownLocks();
  void manageSpeedUpDownLocks();

  void manageButtonsState(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  template <typename T>
  std::unique_ptr<T> calcTwist(const sensor_msgs::msg::Joy::ConstSharedPtr joy);

  template <typename T>
  void publishTwist(std::unique_ptr<T> twist);

  template <typename T>
  void publishZeroTwist();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  double linear_x_;
  double linear_x_max_;
  double angular_z_;
  double angular_z_max_;
  double speed_up_down_linear_x_ = 0;
  double speed_up_down_linear_x_original_;
  double speed_up_down_angular_z_ = 0;
  double speed_up_down_angular_z_original_;
  double speed_up_down_scale_linear_x_;
  double speed_up_down_scale_angular_z_;
  bool publish_twist_stamped_;
  std::string frame_id_;

  bool auto_teleop_mode_ = false;
  bool button_b_state_ = false;
  bool button_lb_state_ = false;
  bool button_rb_state_ = false;

  bool speed_up_linear_x_lock_;
  bool speed_down_linear_x_lock_;
  bool speed_up_angular_z_lock_;
  bool speed_down_angular_z_lock_;
};

enum Button {
  A,
  B,
  X,
  Y,
};

}  // namespace teleop_twist_joy_comfy

#endif  // TELEOP_TWIST_JOY_COMFY__TELEOP_TWIST_JOY_COMFY_HPP_
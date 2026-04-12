#ifndef BUMPERBOT_LOCALIZATION__ODOMETRY_MOTION_MODEL_HPP_
#define BUMPERBOT_LOCALIZATION__ODOMETRY_MOTION_MODEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <random>
#include <string>

class OdometryMotionModel : public rclcpp::Node
{
public:
  explicit OdometryMotionModel(const std::string & name = "odometry_motion_model");

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  int nr_samples_;

  double last_odom_x_;
  double last_odom_y_;
  double last_odom_theta_;
  bool is_first_odom_;

  geometry_msgs::msg::PoseArray samples_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  std::mt19937 rng_;
};

#endif  // BUMPERBOT_LOCALIZATION__ODOMETRY_MOTION_MODEL_HPP_

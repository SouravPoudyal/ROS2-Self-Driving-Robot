#include "bumperbot_localization/odometry_motion_model.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <random>
#include <string>

using std::placeholders::_1;

namespace
{
double normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

OdometryMotionModel::OdometryMotionModel(const std::string & name)
: Node(name),
  alpha1_(0.1),
  alpha2_(0.1),
  alpha3_(0.1),
  alpha4_(0.1),
  nr_samples_(300),
  last_odom_x_(0.0),
  last_odom_y_(0.0),
  last_odom_theta_(0.0),
  is_first_odom_(true),
  rng_(std::random_device{}())
{
  declare_parameter("alpha1", 0.1);
  declare_parameter("alpha2", 0.1);
  declare_parameter("alpha3", 0.1);
  declare_parameter("alpha4", 0.1);
  declare_parameter("nr_samples", 300);

  alpha1_ = get_parameter("alpha1").as_double();
  alpha2_ = get_parameter("alpha2").as_double();
  alpha3_ = get_parameter("alpha3").as_double();
  alpha4_ = get_parameter("alpha4").as_double();
  nr_samples_ = get_parameter("nr_samples").as_int();

  if (nr_samples_ <= 0) {
    RCLCPP_FATAL_STREAM(get_logger(), "Invalid number of samples requested: " << nr_samples_);
    throw std::runtime_error("nr_samples must be > 0");
  }

  samples_.poses.resize(static_cast<size_t>(nr_samples_));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "bumperbot_controller/odom",
    10,
    std::bind(&OdometryMotionModel::odomCallback, this, _1));

  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    "bumperbot_controller/samples",
    10);
}

void OdometryMotionModel::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  tf2::Quaternion odom_q(
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z,
    odom->pose.pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(odom_q).getRPY(roll, pitch, yaw);

  const double odom_x = odom->pose.pose.position.x;
  const double odom_y = odom->pose.pose.position.y;
  const double odom_theta = yaw;

  // Initialize all particles at the first odometry pose
  if (is_first_odom_) {
    last_odom_x_ = odom_x;
    last_odom_y_ = odom_y;
    last_odom_theta_ = odom_theta;

    samples_.header.frame_id = odom->header.frame_id;
    samples_.header.stamp = odom->header.stamp;

    tf2::Quaternion init_q;
    init_q.setRPY(0.0, 0.0, odom_theta);
    init_q.normalize();

    for (auto & sample : samples_.poses) {
      sample.position.x = odom_x;
      sample.position.y = odom_y;
      sample.position.z = 0.0;
      sample.orientation.x = init_q.x();
      sample.orientation.y = init_q.y();
      sample.orientation.z = init_q.z();
      sample.orientation.w = init_q.w();
    }

    is_first_odom_ = false;
    pose_array_pub_->publish(samples_);
    return;
  }

  // Odometry motion decomposition from Probabilistic Robotics
  const double dx = odom_x - last_odom_x_;
  const double dy = odom_y - last_odom_y_;
  const double delta_trans = std::sqrt(dx * dx + dy * dy);

  double delta_rot1 = 0.0;
  if (delta_trans > 1e-12) {
    delta_rot1 = normalize(std::atan2(dy, dx) - last_odom_theta_);
  }

  const double delta_rot2 = normalize(odom_theta - last_odom_theta_ - delta_rot1);

  // Variances from Probabilistic Robotics
  const double rot1_var =
    alpha1_ * delta_rot1 * delta_rot1 +
    alpha2_ * delta_trans * delta_trans;

  const double trans_var =
    alpha3_ * delta_trans * delta_trans +
    alpha4_ * delta_rot1 * delta_rot1 +
    alpha4_ * delta_rot2 * delta_rot2;

  const double rot2_var =
    alpha1_ * delta_rot2 * delta_rot2 +
    alpha2_ * delta_trans * delta_trans;

  const double rot1_std = std::sqrt(rot1_var);
  const double trans_std = std::sqrt(trans_var);
  const double rot2_std = std::sqrt(rot2_var);

  std::normal_distribution<double> rot1_noise(0.0, rot1_std);
  std::normal_distribution<double> trans_noise(0.0, trans_std);
  std::normal_distribution<double> rot2_noise(0.0, rot2_std);

  for (auto & sample : samples_.poses) {
    tf2::Quaternion sample_q(
      sample.orientation.x,
      sample.orientation.y,
      sample.orientation.z,
      sample.orientation.w);

    double sample_roll, sample_pitch, sample_yaw;
    tf2::Matrix3x3(sample_q).getRPY(sample_roll, sample_pitch, sample_yaw);

    const double delta_rot1_hat = delta_rot1 - rot1_noise(rng_);
    const double delta_trans_hat = delta_trans - trans_noise(rng_);
    const double delta_rot2_hat = delta_rot2 - rot2_noise(rng_);

    sample.position.x += delta_trans_hat * std::cos(sample_yaw + delta_rot1_hat);
    sample.position.y += delta_trans_hat * std::sin(sample_yaw + delta_rot1_hat);
    sample.position.z = 0.0;

    const double new_theta = normalize(sample_yaw + delta_rot1_hat + delta_rot2_hat);

    tf2::Quaternion new_q;
    new_q.setRPY(0.0, 0.0, new_theta);
    new_q.normalize();

    sample.orientation.x = new_q.x();
    sample.orientation.y = new_q.y();
    sample.orientation.z = new_q.z();
    sample.orientation.w = new_q.w();
  }

  last_odom_x_ = odom_x;
  last_odom_y_ = odom_y;
  last_odom_theta_ = odom_theta;

  samples_.header.frame_id = odom->header.frame_id;
  samples_.header.stamp = odom->header.stamp;
  pose_array_pub_->publish(samples_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryMotionModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

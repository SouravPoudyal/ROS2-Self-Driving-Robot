#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

class TrajectoryPlotter : public rclcpp::Node
{
    public:
        TrajectoryPlotter() : Node("trajectory_plotter")
        {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/bumperbot_controller/odom",
                 10, std::bind(&TrajectoryPlotter::odom_callback, this, _1));
            path_pub_ = create_publisher<nav_msgs::msg::Path>("/bumperbot_controller/Path", 10);

        }
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        nav_msgs::msg::Path path_;

        void odom_callback(const nav_msgs::msg::Odometry & msg)
        {
            path_.header.frame_id = msg.header.frame_id;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = msg.header.frame_id;
            pose.header.stamp = msg.header.stamp;
            pose.pose = msg.pose.pose;

            path_.poses.push_back(pose);
            path_.header.stamp = msg.header.stamp;


            path_pub_->publish(path_);
        }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlotter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


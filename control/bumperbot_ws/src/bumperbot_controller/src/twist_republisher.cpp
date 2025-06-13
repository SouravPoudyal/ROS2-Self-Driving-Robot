#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class TwistRepublisher: public rclcpp::Node
{
    public:
        TwistRepublisher() : Node("twist_republisher")
        {
            pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10);
            sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&TwistRepublisher::twistStampedCallback, this, _1) );
        }
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;

        void twistStampedCallback(const geometry_msgs::msg::Twist &msg)
        {
            geometry_msgs::msg::TwistStamped twist_stamped_msg;

            twist_stamped_msg.header.stamp = this->get_clock()->now();
            twist_stamped_msg.header.frame_id = "base_link";

            twist_stamped_msg.twist = msg;

            pub_->publish(twist_stamped_msg);            

        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
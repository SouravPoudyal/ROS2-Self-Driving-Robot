#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        // Create a publisher for the /cmd_vel topic
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Start the keyboard input thread
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardTeleop::keyboardInput, this));
    }

private:
    void keyboardInput()
    {
        geometry_msgs::msg::Twist twist_msg;

        // Get a character from the user using the terminal
        char key = getKey();

        // Handle the key input and set the corresponding movement values
        switch (key)
        {
        case 'w':
            twist_msg.linear.x = 1.0;  // Move forward
            break;
        case 's':
            twist_msg.linear.x = -1.0; // Move backward
            break;
        case 'a':
            twist_msg.angular.z = 1.0;  // Turn left
            break;
        case 'd':
            twist_msg.angular.z = -1.0; // Turn right
            break;
        case 'q':
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;  // Stop the robot
            break;
        default:
            break;
        }

        // Publish the twist message to /cmd_vel
        pub_->publish(twist_msg);
    }

    char getKey()
    {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}

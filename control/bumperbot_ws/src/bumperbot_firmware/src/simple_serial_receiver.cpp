#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

#include <chrono>


using namespace std::chrono_literals;

class SimpleSerialRevceiver : public rclcpp::Node
{
public:
  SimpleSerialRevceiver() : Node("simple_serial_receiver")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    port_ = get_parameter("port").as_string();
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    pub_ = create_publisher<std_msgs::msg::String>("simple_receiver", 10);
    timer_ = create_wall_timer(1s, std::bind(&SimpleSerialRevceiver::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
  }

  ~SimpleSerialRevceiver()
  {
    arduino_.Close();
  }

  void timerCallback()
  {
    auto message = std_msgs::msg::String();
    if(rclcpp::ok() && arduino_.IsDataAvailable())
    {
        arduino_.ReadLine(message.data);
    }
    pub_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_;
  LibSerial::SerialPort arduino_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialRevceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
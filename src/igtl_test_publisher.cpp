#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IGTLPublisher : public rclcpp::Node
{
public:
  IGTLPublisher(): Node("minimal_subscriber"), count_(0)
  {
    string_publisher_ = this->create_publisher<ros2_igtl_bridge::msg::String>("IGTL_STRING_OUT", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&IGTLPublisher::timer_callback, this));    
  }

private:

  void timer_callback()
  {
    auto string_msg = ros2_igtl_bridge::msg::String();
    string_msg.name = "Test";
    string_msg.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", string_msg.data.c_str());
    string_publisher_->publish(string_msg);
  }
  
  void string_callback(const ros2_igtl_bridge::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::String");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  }
  
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::String>::SharedPtr string_publisher_;  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLPublisher>());
  rclcpp::shutdown();
  return 0;
}

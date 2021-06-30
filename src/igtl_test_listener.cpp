#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"

using std::placeholders::_1;

class IGTLSubscriber : public rclcpp::Node
{
public:
  IGTLSubscriber(): Node("minimal_subscriber")
  {
    string_subscription_ = this->create_subscription<ros2_igtl_bridge::msg::String>("IGTL_STRING_IN", 10, std::bind(&IGTLSubscriber::string_callback, this, _1));
  }

private:
  void string_callback(const ros2_igtl_bridge::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::String");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  }
  
  rclcpp::Subscription<ros2_igtl_bridge::msg::String>::SharedPtr string_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLSubscriber>());
  rclcpp::shutdown();
  return 0;
}

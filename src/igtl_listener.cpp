#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_subscriber")
  {
    //subscription_ = this->create_subscription<std_msgs::msg::String>("IGTL_STRING_IN", 10, std::bind(&MinimalSubscriber::string_callback, this, _1));
    string_subscription_ = this->create_subscription<ros2_igtl_bridge::msg::String>("IGTL_STRING_IN", 10, std::bind(&MinimalSubscriber::string_callback, this, _1));
  }

private:
  void string_callback(const ros2_igtl_bridge::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  rclcpp::Subscription<ros2_igtl_bridge::msg::String>::SharedPtr string_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

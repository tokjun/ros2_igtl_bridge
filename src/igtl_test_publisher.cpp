#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"
#include "ros2_igtl_bridge/msg/transform.hpp"
#include "ros2_igtl_bridge/msg/point.hpp"
// #include "ros2_igtl_bridge/msg/point_cloud.hpp"
// #include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IGTLPublisher : public rclcpp::Node
{
public:
  IGTLPublisher(): Node("minimal_subscriber"), count_(0)
  {
    string_publisher_      = this->create_publisher<ros2_igtl_bridge::msg::String>("IGTL_STRING_OUT", 10);
    transform_publisher_   = this->create_publisher<ros2_igtl_bridge::msg::Transform>("IGTL_STRING_OUT", 10);
    point_publisher_       = this->create_publisher<ros2_igtl_bridge::msg::Point>("IGTL_STRING_OUT", 10);
    // point_cloud_publisher_ = this->create_publisher<ros2_igtl_bridge::msg::PointCloud>("IGTL_STRING_OUT", 10);
    // image_publisher_       = this->create_publisher<sensor_msgs::msg::Image>("IGTL_STRING_OUT", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&IGTLPublisher::timer_callback, this));    
  }

private:

  void timer_callback()
  {
    // String message
    auto string_msg = ros2_igtl_bridge::msg::String();
    string_msg.name = "test_string";
    string_msg.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", string_msg.data.c_str());
    string_publisher_->publish(string_msg);

    // Transform message
    auto transform_msg   = ros2_igtl_bridge::msg::Transform();
    transform_msg.name = "test_transform";
    transform_msg.transform.translation.x = 10.0;
    transform_msg.transform.translation.y = 20.0;
    transform_msg.transform.translation.z = 30.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;
    transform_msg.transform.rotation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", transform_msg.name.c_str());
    transform_publisher_->publish(transform_msg);

    // Point message
    auto point_msg       = ros2_igtl_bridge::msg::Point();
    point_msg.name = "test_point";
    point_msg.pointdata.x = 40.0;
    point_msg.pointdata.y = 50.0;
    point_msg.pointdata.z = 60.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", point_msg.name.c_str());
    point_publisher_->publish(point_msg);

    // // PointCloud message
    // auto point_cloud_msg = ros2_igtl_bridge::msg::PointCloud();
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", point_cloud_msg.name.c_str());
    // point_cloud_publisher_->publish(point_cloud_msg);

    // // Image message
    // auto image_msg;      = sensor_msgs::msg::Image();
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", image_msg.name.c_str());
    // image_publisher_->publish(image_msg);
    
  }

  size_t count_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<ros2_igtl_bridge::msg::String>::SharedPtr     string_publisher_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::Transform>::SharedPtr  transform_publisher_;
  rclcpp::Publisher<ros2_igtl_bridge::msg::Point>::SharedPtr      point_publisher_;
  // rclcpp::Publisher<ros2_igtl_bridge::msg::PointCloud>::SharedPtr point_cloud_publisher_;  
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           image_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLPublisher>());
  rclcpp::shutdown();
  return 0;
}

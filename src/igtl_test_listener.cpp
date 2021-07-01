#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_igtl_bridge/msg/string.hpp"
#include "ros2_igtl_bridge/msg/transform.hpp"
#include "ros2_igtl_bridge/msg/point.hpp"
// #include "ros2_igtl_bridge/msg/point_cloud.hpp"
// #include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class IGTLSubscriber : public rclcpp::Node
{
public:
  IGTLSubscriber(): Node("minimal_subscriber")
  {
    string_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::String>("IGTL_STRING_IN", 10,
                                                               std::bind(&IGTLSubscriber::string_callback, this, _1));
    transform_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::Transform>("IGTL_TRANSFORM_IN", 10,
                                                               std::bind(&IGTLSubscriber::transform_callback, this, _1));
    point_subscription_ =
      this->create_subscription<ros2_igtl_bridge::msg::Point>("IGTL_POINT_IN", 10,
                                                               std::bind(&IGTLSubscriber::point_callback, this, _1));
    // point_cloud_subscription_ =
    //   this->create_subscription<ros2_igtl_bridge::msg::PointCloudn>("IGTL_POINT_CLOUD_IN", 10,
    //                                                            std::bind(&IGTLSubscriber::point_cloud_callback, this, _1));
    // image_subscription_ =
    //   this->create_subscription<sensor_msgs::msg::Image>("IGTL_IMAGE_IN", 10,
    //                                                            std::bind(&IGTLSubscriber::image_callback, this, _1));
  }

private:
  void string_callback(const ros2_igtl_bridge::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::String");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  }
  
  void transform_callback(const ros2_igtl_bridge::msg::Transform::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::Transform");
    RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    translation : (%f, %f, %f)",
                msg->transform.translation.x,
                msg->transform.translation.y,
                msg->transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "    rotation    : (%f, %f, %f, %f)",
                msg->transform.rotation.x,
                msg->transform.rotation.y,
                msg->transform.rotation.z,
                msg->transform.rotation.w);
  }
  
  void point_callback(const ros2_igtl_bridge::msg::Point::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::Point");
    RCLCPP_INFO(this->get_logger(), "    name        : '%s'", msg->name.c_str());
    RCLCPP_INFO(this->get_logger(), "    point : (%f, %f, %f)",
                msg->pointdata.x,
                msg->pointdata.y,
                msg->pointdata.z);
  }
  
  // void point_cloud_callback(const ros2_igtl_bridge::msg::PointCloud::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received ros2_igtl_bridge::msg::Point");
  //   RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
  //   RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  // }
  // 
  // void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received sensor_msgs::msg::Image::SharedPtr");
  //   RCLCPP_INFO(this->get_logger(), "    name: '%s'", msg->name.c_str());
  //   RCLCPP_INFO(this->get_logger(), "    data: '%s'", msg->data.c_str());    
  // }

  rclcpp::Subscription<ros2_igtl_bridge::msg::String>::SharedPtr string_subscription_;
  rclcpp::Subscription<ros2_igtl_bridge::msg::Transform>::SharedPtr transform_subscription_;
  rclcpp::Subscription<ros2_igtl_bridge::msg::Point>::SharedPtr point_subscription_;
  // rclcpp::Subscription<ros2_igtl_bridge::msg::PointCloud>::SharedPtr point_cloud_subscription_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLSubscriber>());
  rclcpp::shutdown();
  return 0;
}

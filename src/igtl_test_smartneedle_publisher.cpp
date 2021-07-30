#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IGTLPublisher : public rclcpp::Node
{
public:
  IGTLPublisher(): Node("minimal_subscriber"), count_(0)
  {
    skin_entry_publisher_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/subject/state/skin_entry", 10);
    target_publisher_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/subject/state/target", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&IGTLPublisher::timer_callback, this));    
  }

private:

  void timer_callback()
  {
    // Point message
    auto se_point_msg       = geometry_msgs::msg::PointStamped();
    se_point_msg.point.x = 40.0;
    se_point_msg.point.y = 50.0;
    se_point_msg.point.z = 60.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: Skin Entry Point.");
    skin_entry_publisher_->publish(se_point_msg);

    auto t_point_msg       = geometry_msgs::msg::PointStamped();
    t_point_msg.point.x = 40.0;
    t_point_msg.point.y = 50.0;
    t_point_msg.point.z = 60.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: Target Point.");
    target_publisher_->publish(t_point_msg);

  }

  size_t count_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr     skin_entry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr     target_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLPublisher>());
  rclcpp::shutdown();
  return 0;
}

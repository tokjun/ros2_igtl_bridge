#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using std::placeholders::_1;

class IGTLSubscriber : public rclcpp::Node
{
public:
  IGTLSubscriber(): Node("minimal_subscriber")
  {
    state_pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>("/stage/state/pose", 10,
                                                                 std::bind(&IGTLSubscriber::state_pose_callback, this, _1));
    state_needle_pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>("/stage/state/needle_pose", 10,
                                                               std::bind(&IGTLSubscriber::state_needle_pose_callback, this, _1));
    cmd_pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>("/stage/cmd/pose", 10,
                                                               std::bind(&IGTLSubscriber::cmd_pose_callback, this, _1));
    state_needle_shape_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseArray>("/needle/state/shape", 10,
                                                               std::bind(&IGTLSubscriber::state_needle_shape_callback, this, _1));
  }

private:
  
  void state_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /stage/state/pose");
    RCLCPP_INFO(this->get_logger(), "    position    : (%f, %f, %f)",
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "    orientation : (%f, %f, %f, %f)",
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
  }
  
  void state_needle_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /stage/state/needle_pose");
    RCLCPP_INFO(this->get_logger(), "    position    : (%f, %f, %f)",
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "    orientation : (%f, %f, %f, %f)",
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
  }
  
  void cmd_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /stage/cmd/pose");
    RCLCPP_INFO(this->get_logger(), "    position    : (%f, %f, %f)",
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "    orientation : (%f, %f, %f, %f)",
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
  }
  
  void state_needle_shape_callback(const geometry_msgs::msg::PoseArray::SharedPtr   msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /needle/state/shape");
    int n = msg->poses.size();
    for (int i = 0; i < n; i ++) {
      RCLCPP_INFO(this->get_logger(), "    Pose [%d]", i);
      RCLCPP_INFO(this->get_logger(), "    position    : (%f, %f, %f)",
                  msg->poses[i].position.x,
                  msg->poses[i].position.y,
                  msg->poses[i].position.z);
      RCLCPP_INFO(this->get_logger(), "    orientation : (%f, %f, %f, %f)",
                  msg->poses[i].orientation.x,
                  msg->poses[i].orientation.y,
                  msg->poses[i].orientation.z,
                  msg->poses[i].orientation.w);
    }
    
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_needle_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr state_needle_shape_subscription_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IGTLSubscriber>());
  rclcpp::shutdown();
  return 0;
}

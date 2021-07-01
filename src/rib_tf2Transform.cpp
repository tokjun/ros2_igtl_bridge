#include <ros2_igtl_bridge/msg/transform.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <ros2_igtl_bridge/msg/transform.hpp>

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node{

private:
  void timer_callback(){

    try {
      std::string base_link("base_link");
      geometry_msgs::msg::TransformStamped transform_stamped;
      ros2_igtl_bridge::msg::Transform Rt;

      // couldn't figure how to do several publish per callback. (cannot add another spinner to this object);
      switch( cnt ){
      case 0:{
	transform_stamped = buffer_.lookupTransform( base_link, "shoulder_link", tf2::TimePoint());
	Rt.name = "shoulder_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      case 1:{
	transform_stamped = buffer_.lookupTransform( base_link, "upper_arm_link", tf2::TimePoint());
	Rt.name = "upper_arm_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      case 2:{
	transform_stamped = buffer_.lookupTransform( base_link, "forearm_link", tf2::TimePoint());
	Rt.name = "forearm_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      case 3:{
	transform_stamped = buffer_.lookupTransform( base_link, "wrist_1_link", tf2::TimePoint());
	Rt.name = "wrist_1_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      case 4:{
	transform_stamped = buffer_.lookupTransform( base_link, "wrist_2_link", tf2::TimePoint());
	Rt.name = "wrist_2_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      case 5:{
	transform_stamped = buffer_.lookupTransform( base_link, "wrist_3_link", tf2::TimePoint());
	Rt.name = "wrist_3_link";
	Rt.transform = transform_stamped.transform;
	publisher_->publish(Rt);
	break;
      }
      }
      cnt++;
      if( cnt==6 ) cnt=0;
    }
    catch(const tf2::TransformException& ex){}
  }

public:
  
  TFListener() :
    Node("tf_listener"),
    buffer_( get_clock() ),
    shared_ptr( this ),
    cnt(0){
    
    timer_ = this->create_wall_timer(20ms, std::bind(&TFListener::timer_callback, this));
    publisher_ =
      this->create_publisher<ros2_igtl_bridge::msg::Transform>("IGTL_TRANSFORM_OUT", 10);
    
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    
  }

  rclcpp::Publisher<ros2_igtl_bridge::msg::Transform>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  tf2_ros::Buffer buffer_;
  std::shared_ptr<TFListener> shared_ptr;
  int cnt;

};

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = std::make_shared<TFListener>();

  rclcpp::spin( nh );
  
  rclcpp::shutdown();

  return 0;

}


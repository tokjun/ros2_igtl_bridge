#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtl_node.h"

#include "rib_converter_manager.h"
#include "rib_converter_point.h"
#include "rib_converter_pointcloud.h"
#include "rib_converter_transform.h"
#include "rib_converter_polydata.h"
#include "rib_converter_string.h"
#include "rib_converter_image.h"

using namespace std::chrono_literals;


OpenIGTLinkNode::OpenIGTLinkNode() : Node(IGTL_DEFAULT_NODE_NAME), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
  this->addConverters();
}

OpenIGTLinkNode::OpenIGTLinkNode(const std::string nodeName) : Node(nodeName), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
  this->addConverters();
}


void OpenIGTLinkNode::addConverters()
{
  this->converterManager = new RIBConverterManager;
  rclcpp::Node::SharedPtr ptr = shared_from_this();
  this->converterManager->setNode(ptr);

  // Regisgter converter classes
  RIBConverterPoint * point = new RIBConverterPoint;
  RIBConverterTransform* transform = new RIBConverterTransform;
  RIBConverterPolyData* polydata = new RIBConverterPolyData;
  RIBConverterString* string = new RIBConverterString;
  RIBConverterImage* image = new RIBConverterImage;
  RIBConverterPointCloud* pointcloud = new RIBConverterPointCloud;
  
  this->converterManager->AddConverter(point, 10, "IGTL_POINT_IN", "IGTL_POINT_OUT");
  this->converterManager->AddConverter(transform, 10, "IGTL_TRANSFORM_IN", "IGTL_TRANSFORM_OUT");
  this->converterManager->AddConverter(polydata, 10, "IGTL_POLYDATA_IN", "IGTL_POLYDATA_OUT");
  this->converterManager->AddConverter(string, 10, "IGTL_STRING_IN", "IGTL_STRING_OUT");
  this->converterManager->AddConverter(image, 10, "IGTL_IMAGE_IN", "IGTL_IMAGE_OUT");
  this->converterManager->AddConverter(pointcloud, 10, "IGTL_POINTCLOUD_IN", "IGTL_POINTCLOUD_OUT");

}


void OpenIGTLinkNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}



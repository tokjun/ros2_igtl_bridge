#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtl_node.h"

using namespace std::chrono_literals;


OpenIGTLinkNode::OpenIGTLinkNode() : Node(IGTL_DEFAULT_NODE_NAME), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
  this->AddConverters();
}

OpenIGTLinkNode::OpenIGTLinkNode(const std::string nodeName) : Node(nodeName), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&OpenIGTLinkNode::timer_callback, this));
  this->AddConverters();
}


void OpenIGTLinkNode::AddConverters()
{
  this->converterManager = new RIBConverterManager;
  rclcpp::Node::SharedPtr ptr = shared_from_this();
  this->converterManager->setNode(ptr);
}







void OpenIGTLinkNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}



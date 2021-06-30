/*=========================================================================

  Program:   Converter Class for Transform
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterTransform_H
#define __RIBConverterTransform_H

#include "rib_converter.h"

// ROS header files
#include "rclcpp/rclcpp.hpp"

// ROS message header files
#include "ros2_igtl_bridge/msg/transform.hpp"
#include "geometry_msgs/msg/transform.h"

// OpenIGTLink message files
#include "igtlTransformMessage.h"

class RIBConverterTransform : public RIBConverter<ros2_igtl_bridge::msg::Transform>
{

public:
  RIBConverterTransform();
  RIBConverterTransform(rclcpp::Node::SharedPtr n);
  RIBConverterTransform(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "TRANSFORM"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
protected:
  virtual void onROSMessage(const ros2_igtl_bridge::msg::Transform::SharedPtr msg);
};


#endif // __RIBConverterTransform_H



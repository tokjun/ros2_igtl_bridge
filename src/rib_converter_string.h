/*=========================================================================

  Program:   Converter Class for String
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterString_H
#define __RIBConverterString_H

#include "rib_converter.h"

// ROS header files
//#include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

// ROS message header files
#include "ros2_igtl_bridge/msg/string.hpp"
//#include "std_msgs/String.h"

// OpenIGTLink message files
#include "igtlMessageHeader.h"

class RIBConverterString : public RIBConverter<ros2_igtl_bridge::msg::String>
{

public:
  RIBConverterString();
  RIBConverterString(rclcpp::Node::SharedPtr n);
  RIBConverterString(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "STRING"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
protected:
  virtual void onROSMessage(const ros2_igtl_bridge::msg::String::SharedPtr msg);
};


#endif // __RIBConverterString_H



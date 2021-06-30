/*=========================================================================

  Program:   Converter Class for Point
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterPoint_H
#define __RIBConverterPoint_H

#include "rib_converter.h"

// ROS header files
#include "rclcpp/rclcpp.hpp"

// ROS message header files
#include "ros2_igtl_bridge/msg/point.hpp"

// OpenIGTLink message files
#include "igtlStringMessage.h"


class RIBConverterPoint : public RIBConverter<ros2_igtl_bridge::msg::Point>
{

public:
  RIBConverterPoint();
  RIBConverterPoint(rclcpp::Node::SharedPtr n);
  RIBConverterPoint(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "POINT"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  virtual void onROSMessage(const ros2_igtl_bridge::msg::Point::SharedPtr msg);
};

#endif // __RIBConverterPoint_H



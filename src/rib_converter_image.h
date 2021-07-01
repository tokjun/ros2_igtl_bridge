/*=========================================================================

  Program:   Converter Class for Image
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __RIBConverterImage_H
#define __RIBConverterImage_H

#include "rib_converter.h"

// ROS header files
#include "sensor_msgs/msg/image.hpp"

// ROS message header files
//#include "ros2_igtl_bridge/msg/image.hpp"

// OpenIGTLink message files
#include "igtlStringMessage.h"


//class RIBConverterImage : public RIBConverter<ros2_igtl_bridge::msg::Image>
class RIBConverterImage : public RIBConverter<sensor_msgs::msg::Image>
{

public:
  RIBConverterImage();
  RIBConverterImage(rclcpp::Node::SharedPtr n);
  RIBConverterImage(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n=NULL);
  
  //virtual uint32_t queueSizePublish() { return 10; }
  //virtual uint32_t queueSizeSubscribe() { return 10; }
  virtual const char* messageTypeString() { return "IMAGE"; }

public:  
  virtual int onIGTLMessage(igtl::MessageHeader * header);
 protected:
  //virtual void onROSMessage(const ros2_igtl_bridge::msg::Image::SharedPtr msg);
  virtual void onROSMessage(const sensor_msgs::msg::Image::SharedPtr msg);
};


#endif // __RIBConverterImage_H



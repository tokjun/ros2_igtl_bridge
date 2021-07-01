/*=========================================================================

  Program:   Converter Class for Point
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

  =========================================================================*/

#include "rib_converter_point.h"
#include "rib_converter_manager.h"
#include "rclcpp/rclcpp.hpp"
#include "igtlPointMessage.h"
#include "geometry_msgs/msg/point.hpp"

RIBConverterPoint::RIBConverterPoint()
  : RIBConverter<ros2_igtl_bridge::msg::Point>()
{
}

RIBConverterPoint::RIBConverterPoint(rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::Point>(n)
{
}

RIBConverterPoint::RIBConverterPoint(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::Point>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterPoint::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetMessageHeader(header);
  pointMsg->AllocatePack();

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  bool timeout = false;
  socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize(), timeout);
  int c = pointMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to unpack the message. Datatype: POINT.");
    return 0;
    }
  
  int npoints = pointMsg->GetNumberOfPointElement ();
  
  if (npoints > 0)
    {
    for (int i = 0; i < npoints; i ++)
      {
      igtlFloat32 point[3];
      igtl::PointElement::Pointer elem = igtl::PointElement::New();
      pointMsg->GetPointElement (i,elem);
      elem->GetPosition(point);
      
      ros2_igtl_bridge::msg::Point msg;
      
      msg.pointdata.x = point[0];
      msg.pointdata.y = point[1];
      msg.pointdata.z = point[2];
      msg.name = elem->GetName();
      
      this->publisher->publish(msg);
      }
    }
  else
    {
    RCLCPP_ERROR(this->node->get_logger(), "Message POINT is empty");
    return 0;
    }
  
  return 1;
  
}

void RIBConverterPoint::onROSMessage(const ros2_igtl_bridge::msg::Point::SharedPtr msg)
{
  
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  std::cout<< "onROSMessage (Point): "<< msg->name <<std::endl;
  
  if (socket.IsNull())
    {
    return;
    }

  geometry_msgs::msg::Point point = msg->pointdata;
  
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());
  
  igtl::PointElement::Pointer pointE; 
  pointE = igtl::PointElement::New();
  pointE->SetPosition(point.x, point.y,point.z);
  pointMsg->AddPointElement(pointE);
  pointMsg->Pack();
  
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());
}


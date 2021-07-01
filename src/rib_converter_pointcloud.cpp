/*=========================================================================

  Program:   Converter Class for Point Cloud
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_pointcloud.h"
#include "rib_converter_manager.h"
#include "igtlPointMessage.h"


RIBConverterPointCloud::RIBConverterPointCloud()
  : RIBConverter<ros2_igtl_bridge::msg::PointCloud>()
{
}

RIBConverterPointCloud::RIBConverterPointCloud(rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PointCloud>(n)
{
}

RIBConverterPointCloud::RIBConverterPointCloud(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PointCloud>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterPointCloud::onIGTLMessage(igtl::MessageHeader * header)
{
  
}

void RIBConverterPointCloud::onROSMessage(const ros2_igtl_bridge::msg::PointCloud::SharedPtr msg)
{

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }

  int pcl_size = msg->pointdata.size();
  if (!pcl_size) 
    {
    RCLCPP_ERROR(this->node->get_logger(), "[ROS-IGTL-Bridge] Pointcloud is empty!");
    return;
    }
  
  igtl::PointMessage::Pointer pointMsg = igtl::PointMessage::New();
  pointMsg->SetDeviceName(msg->name.c_str());
  
  for (int i = 0; i<pcl_size;i++)
    {
    igtl::PointElement::Pointer pointE; 
    pointE = igtl::PointElement::New();
    pointE->SetPosition(msg->pointdata[i].x, msg->pointdata[i].y,msg->pointdata[i].z);		
    pointMsg->AddPointElement(pointE);
    }
  pointMsg->Pack();
  socket->Send(pointMsg->GetPackPointer(), pointMsg->GetPackSize());

}






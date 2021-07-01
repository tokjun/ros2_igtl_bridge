/*=========================================================================

  Program:   Converter Class for Image
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rclcpp/rclcpp.hpp"

#include "rib_converter_image.h"
#include "rib_converter_manager.h"

#include "igtlImageMessage.h"

RIBConverterImage::RIBConverterImage()
//: RIBConverter<ros2_igtl_bridge::msg::Image>()
  : RIBConverter<sensor_msgs::msg::Image>()
{
}

RIBConverterImage::RIBConverterImage(rclcpp::Node::SharedPtr n)
//: RIBConverter<ros2_igtl_bridge::msg::Image>(n)
  : RIBConverter<sensor_msgs::msg::Image>(n)
{
}

RIBConverterImage::RIBConverterImage(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
//  : RIBConverter<ros2_igtl_bridge::msg::Image>(topicPublish, topicSubscribe, n)
  : RIBConverter<sensor_msgs::msg::Image>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterImage::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetMessageHeader(header);
  imgMsg->AllocatePack();
  
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  bool timeout = false;
  socket->Receive(imgMsg->GetPackBodyPointer(), imgMsg->GetPackBodySize(), timeout);
  int c = imgMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    RCLCPP_ERROR(this->node->get_logger(), "[ROS-IGTL-Failed to unpack the message. Datatype: IMAGE.");
    return 0;
    }
  
  std::vector<uint8_t> image;
  sensor_msgs::msg::Image::UniquePtr img_msg(new sensor_msgs::msg::Image());
  
  float spacing[] = {0,0,0};
  imgMsg->GetSpacing(spacing);
  
  int   imgsize[] = {0,0,0};
  imgMsg->GetDimensions(imgsize);
  img_msg->height = imgsize[1];
  img_msg->width = imgsize[0];
  
  RCLCPP_INFO(this->node->get_logger(), "h %d",img_msg->height);
  RCLCPP_INFO(this->node->get_logger(), "w %d",img_msg->width);
  
  img_msg->encoding = "mono8";
  img_msg->is_bigendian = false;
  img_msg->step = imgsize[0];
  RCLCPP_INFO(this->node->get_logger(), "s %d",img_msg->step);
  size_t size = img_msg->step* img_msg->height;
  img_msg->data.resize(size);
  
  memcpy((char*)(&img_msg->data[0]),imgMsg->GetScalarPointer(),size);
  
  this->publisher->publish(std::move(img_msg));

  return 1;

}

//void RIBConverterImage::onROSMessage(const ros2_igtl_bridge::msg::Image::SharedPtr msg)
void RIBConverterImage::onROSMessage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  //int   size[]     = {msg->z_steps,msg->y_steps,msg->x_steps};       // image dimension
  //float spacing[]  = {msg->z_spacing,msg->y_spacing,msg->x_spacing};     // spacing (mm/pixel)
  int   size[]     = {(int)msg->width,(int)msg->height,1};       // image dimension
  float spacing[]  = {1.0,1.0,1.0};     // spacing (mm/pixel) 
  
  int   scalarType = igtl::ImageMessage::TYPE_UINT8;

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return;
    }
  
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetCoordinateSystem(2);
  //imgMsg->SetDeviceName(msg->name.c_str());
  imgMsg->SetDeviceName("ROS Image");
  imgMsg->SetOrigin(0,0,0);
  imgMsg->AllocateScalars();
  //-----------------------------------------------------------
  
  //memcpy(imgMsg->GetScalarPointer(),(char*)(&msg->data[0]),msg->data.size());
  memcpy(imgMsg->GetScalarPointer(),(char*)(&msg->data[0]),msg->data.size());
  
  igtl::Matrix4x4 matrixa;
  igtl::IdentityMatrix(matrixa);
  imgMsg->SetMatrix(matrixa);
  
  //------------------------------------------------------------
  // Pack and send
  imgMsg->Pack();
  
  socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}




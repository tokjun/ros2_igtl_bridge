/*=========================================================================

  Program:   Converter Class for PoseArray
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

  =========================================================================*/

#include "rib_converter_posearray.h"
#include "rib_converter_manager.h"
#include "rclcpp/rclcpp.hpp"
#include "igtlTrackingDataMessage.h"
#include "igtlMath.h"
#include "ros2_igtl_bridge/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

RIBConverterPoseArray::RIBConverterPoseArray()
  : RIBConverter<ros2_igtl_bridge::msg::PoseArray>()
{
}

RIBConverterPoseArray::RIBConverterPoseArray(rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PoseArray>(n)
{
}

RIBConverterPoseArray::RIBConverterPoseArray(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::PoseArray>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterPoseArray::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::TrackingDataMessage::Pointer trackingMsg = igtl::TrackingDataMessage::New();
  trackingMsg->SetMessageHeader(header);
  trackingMsg->AllocatePack();

  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  bool timeout = false;
  socket->Receive(trackingMsg->GetPackBodyPointer(), trackingMsg->GetPackBodySize(), timeout);
  int c = trackingMsg->Unpack(1);
  
  if ((c & igtl::MessageHeader::UNPACK_BODY) == 0) 
    {
    RCLCPP_ERROR(this->node->get_logger(), "Failed to unpack the message. Datatype: TDATA.");
    return 0;
    }
  
  int npoints = trackingMsg->GetNumberOfTrackingDataElements();
  
  if (npoints > 0)
    {
    ros2_igtl_bridge::msg::PoseArray msg;
    msg.name = trackingMsg->GetDeviceName();
    msg.posearray.poses.resize(npoints);
    for (int i = 0; i < npoints; i ++)
      {
      igtl::TrackingDataElement::Pointer elem = igtl::TrackingDataElement::New();
      trackingMsg->GetTrackingDataElement (i,elem);
      
      igtl::Matrix4x4 matrix;      
      elem->GetMatrix(matrix);
      
      float q[4];
      igtl::MatrixToQuaternion(matrix, q);
      std::string elemName = "";
      
      msg.posearray.poses[i].position.x = matrix[0][3];
      msg.posearray.poses[i].position.y = matrix[1][3];
      msg.posearray.poses[i].position.z = matrix[2][3];
      
      msg.posearray.poses[i].orientation.x = q[0];
      msg.posearray.poses[i].orientation.y = q[1];
      msg.posearray.poses[i].orientation.z = q[2];
      msg.posearray.poses[i].orientation.w = q[3];
      elemName = "POSE_" + i;
      msg.name = elemName.c_str();
      }
    this->publisher->publish(msg);
    }
  else
    {
    RCLCPP_ERROR(this->node->get_logger(), "Message TDATA is empty");
    return 0;
    }
  
  return 1;
  
}

void RIBConverterPoseArray::onROSMessage(const ros2_igtl_bridge::msg::PoseArray::SharedPtr msg)
{
  
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  std::cout<< "onROSMessage (PoseArray): "<< msg->name <<std::endl;
  
  if (socket.IsNull())
    {
    return;
    }

  geometry_msgs::msg::PoseArray posearray = msg->posearray;
  
  igtl::TrackingDataMessage::Pointer trackingMsg = igtl::TrackingDataMessage::New();
  trackingMsg->SetDeviceName(msg->name.c_str());

  int nposes = msg->posearray.poses.size();

  // We use a std::vector for igtl::TrackingDataElement::Pointer here,
  // because the pointer is declared in the for() loop, the smart pointer will automatically
  // delete the instance at the end of each iteration. The instances must be kept until
  // trackingMsg->Pack() is called.
  
  std::vector< igtl::TrackingDataElement::Pointer > te;
  te.resize(nposes);
  
  for (int i = 0; i < nposes; i ++)
    {  
    te[i] = igtl::TrackingDataElement::New();
    te[i]->SetName("Channel 0");
    igtl::Matrix4x4 matrix;
    
    float q[4];
    q[0] = msg->posearray.poses[i].orientation.x;
    q[1] = msg->posearray.poses[i].orientation.y;
    q[2] = msg->posearray.poses[i].orientation.z;
    q[3] = msg->posearray.poses[i].orientation.w;
    igtl::QuaternionToMatrix(q, matrix);
    
    matrix[0][3] = msg->posearray.poses[i].position.x;
    matrix[1][3] = msg->posearray.poses[i].position.y;
    matrix[2][3] = msg->posearray.poses[i].position.z;
    
    te[i]->SetMatrix(matrix);
    trackingMsg->AddTrackingDataElement(te[i]);
    }
  trackingMsg->Pack();
  socket->Send(trackingMsg->GetPackPointer(), trackingMsg->GetPackSize());
  
}


/*=========================================================================

  Program:   Converter Class for String
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include "rib_converter_string.h"
#include "rib_converter_manager.h"
//#include "ros/ros.h"
#include "igtlStringMessage.h"

RIBConverterString::RIBConverterString()
  : RIBConverter<ros2_igtl_bridge::msg::String>()
{
}

RIBConverterString::RIBConverterString(rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::String>(n)
{
}

RIBConverterString::RIBConverterString(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverter<ros2_igtl_bridge::msg::String>(topicPublish, topicSubscribe, n)
{
}

int RIBConverterString::onIGTLMessage(igtl::MessageHeader * header)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  if (socket.IsNull())
    {
    return 0;
    }

  // Create a message buffer to receive string data
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();

  // Receive string data from the socket
  bool timeout = false;
  socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize(), timeout);

  int b = stringMsg->Unpack(1);
  ros2_igtl_bridge::msg::String msg;
	
  if (b & igtl::MessageHeader::UNPACK_BODY) 
    {
    //std::cout<< "Received String: "<<stringMsg->GetString()<<std::endl;     
    msg.name = stringMsg->GetDeviceName();
    msg.data = stringMsg->GetString();
    // TODO
    //this->publisher.publish(msg);
    return 1;
    }
  else 
    {
    std::cerr << "CRC Check failed!" << std::endl;
    return 0;
    }
}

void RIBConverterString::onROSMessage(const ros2_igtl_bridge::msg::String::SharedPtr msg)
{
  igtl::Socket::Pointer socket = this->manager->GetSocket();
  std::cout<< "onROSMessage: "<< msg->data <<std::endl;     
  
  if (socket.IsNull())
    {
    return;
    }

  //  SendString(msg->name.c_str(), msg->data);
  igtl::StringMessage::Pointer stringMsg = igtl::StringMessage::New();
  stringMsg->SetDeviceName(msg->name.c_str());
  stringMsg->SetString((msg->data).c_str());  
  stringMsg->Pack();
  //std::cout<<stringmsg.c_str()<< " sent"<<std::endl;
  socket->Send(stringMsg->GetPackPointer(), stringMsg->GetPackSize());
}



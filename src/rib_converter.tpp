/*=========================================================================

  Program:   Converter Base Class
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

  =========================================================================*/

#ifndef __RIBConverterPoint_TXX
#define __RIBConverterPoint_TXX

//#include <boost/bind>

template <typename MessageType>
RIBConverter<MessageType>::RIBConverter()
  : RIBConverterBase()
{
}

template <typename MessageType>
RIBConverter<MessageType>::RIBConverter(rclcpp::Node::SharedPtr n)
  : RIBConverterBase(n)
{
}

template <typename MessageType>
RIBConverter<MessageType>::RIBConverter(const char* topicPublish, const char* topicSubscribe, rclcpp::Node::SharedPtr n)
  : RIBConverterBase(topicPublish, topicSubscribe, n)
{
}

template <typename MessageType>
RIBConverter<MessageType>::~RIBConverter()
{
}

template <typename MessageType>
bool RIBConverter<MessageType>::publish(const char* topic)
{
  if (!this->node)
    {
    return false;
    }
  
  if (topic != NULL)
    {
    this->topicPublish = topic;
    }

  // TODO
  //this->publisher = this->node->advertise<MessageType>(this->topicPublish, this->queueSize);
  
  return true;
}

  
template <typename MessageType>
bool RIBConverter<MessageType>::subscribe(const char* topic)
{
  if (!this->node)
    {
    return false;
    }
  
  if (topic != NULL)
    {
    this->topicSubscribe = topic;
    }

  /* TODO
  this->options =
    ros::SubscribeOptions::create<MessageType>(
                                               this->topicSubscribe,
                                               this->queueSize, // queue length
                                               boost::bind(&RIBConverter<MessageType>::onROSMessage, this, _1),
                                               ros::VoidPtr(), // tracked object, we don't need one thus NULL
                                               this->queue // pointer to callback queue object
                                               );
  this->subscriber = this->node->subscribe(options);
  */

  return true;
}

//template <typename MessageType>
//void RIBConverterBase<MessageType>::setup(ros::NodeHandle* nh, igtl::Socket * socket, uint32_t queuSize)
//{
//  this->setNodeHandle(nh);
//  this->setSocket(socket);
//  this->setQueueSize(queueSize);
//}
//template <typename MessageType>
//void RIBConverterBase<MessageType>::setup(ros::NodeHandle* nh, igtl::Socket * socket, const char* topicSubscribe, const char* topicPublish)
//{
//  this->node = nh;
//  this->socket = socket;
//  this->topicSubscribe = topicSubscribe;
//  this->topicPublish = topicPublish;
//}

#endif // __RIBConverterPoint_TXX


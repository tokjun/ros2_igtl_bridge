#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "igtlSocket.h"
#include "igtlClientSocket.h"
#include "igtlServerSocket.h"
#include "rib_converter_manager.h"

#define IGTL_DEFAULT_NODE_NAME "igtl_node"

using namespace std::chrono_literals;

class OpenIGTLinkNode : public rclcpp::Node
{
  
protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

public:
  OpenIGTLinkNode();
  OpenIGTLinkNode(const std::string nodeName);

  void addConverters();
  
  igtl::Socket::Pointer GetSocketPointer();
  
protected:
  virtual int StartIGTLServer();
  virtual int ConnectToIGTLServer();
  virtual void IGTLThread();

  igtl::Socket::Pointer socket;
  igtl::ServerSocket::Pointer serverSocket;
  igtl::ClientSocket::Pointer clientSocket;

  std::thread igtlThread;
  
  //ros::NodeHandle *nh;
  RIBConverterManager * converterManager;
  
  bool isServer;   // Socket type for OpenIGTLink connection
  std::string address;
  int port;
  
private:
  void timer_callback();
  
};





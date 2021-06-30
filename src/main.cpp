/*=========================================================================

  Program:   ROS2-OpenIGTLink Bridge 
  Language:  C++

  Copyright (c) Brigham and Women's Hospital. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "igtl_node.h"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OpenIGTLinkNode>("igtl_node");
  //auto node = std::make_shared<OpenIGTLinkNode>();
  node->addConverters();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

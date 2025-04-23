// Copyright (C) 2024 Christian Brommer and Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "rclcpp/rclcpp.hpp"

#ifdef POSE
#include "mars_wrapper_pose.h"
#endif
#ifdef DUALPOSE
#include "mars_wrapper_dualpose.h"
#endif

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  #ifdef POSE
  auto node = std::make_shared<MarsWrapperPose>();
  RCLCPP_INFO(node->get_logger(), "Starting the MaRS Framework");
  rclcpp::spin(node);
  #endif
  #ifdef DUALPOSE
  auto node = std::make_shared<MarsWrapperDualPose>();
  RCLCPP_INFO(node->get_logger(), "Starting the MaRS Framework");
  rclcpp::spin(node);
  #endif

  rclcpp::shutdown();
  return 0;
}

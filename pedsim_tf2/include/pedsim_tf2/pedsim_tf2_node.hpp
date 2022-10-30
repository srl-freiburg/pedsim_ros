
// Copyright 2020 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: jginesclavero jonatan.gines@urjc.es */

/* Mantainer: jginesclavero jonatan.gines@urjc.es */
#ifndef PEDSIMTF2NODE__H
#define PEDSIMTF2NODE__H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/tracked_person.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace geometry_msgs::msg;

namespace pedsim {
class PedsimTF2 : public rclcpp::Node {
public:
  PedsimTF2(const std::string &name);
  void step();

private:
  void agentsCallback(const pedsim_msgs::msg::AgentStates::SharedPtr msg);
  bool getTFfromAgent(pedsim_msgs::msg::AgentState actor, TransformStamped &tf);

  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr sub_;
  pedsim_msgs::msg::AgentStates::SharedPtr states_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double rand_angle;
  bool angle_generated;
};
}; // namespace pedsim

#endif

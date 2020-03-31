/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "rclcpp/rclcpp.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include <functional>
#include <memory>

#include <pedsim_msgs/msg/agent_force.hpp>
#include <pedsim_msgs/msg/agent_group.hpp>
#include <pedsim_msgs/msg/agent_groups.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/line_obstacle.hpp>
#include <pedsim_msgs/msg/line_obstacles.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pedsim_simulator/agentstatemachine.hpp>
#include <pedsim_simulator/config.hpp>
#include <pedsim_simulator/element/agent.hpp>
#include <pedsim_simulator/element/agentgroup.hpp>
#include <pedsim_simulator/element/attractionarea.hpp>
#include <pedsim_simulator/element/obstacle.hpp>
#include <pedsim_simulator/element/waitingqueue.hpp>
#include <pedsim_simulator/element/waypoint.hpp>
#include <pedsim_simulator/scenarioreader.hpp>
#include <pedsim_simulator/scene.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <pedsim_simulator/PedsimSimulatorConfig.hpp>

//using SimConfig = pedsim_simulator::PedsimSimulatorConfig;
using namespace pedsim_msgs::msg;

/// \class Simulator
/// \brief Simulation wrapper
/// \details ROS interface to the scene object provided by pedsim
class Simulator : public rclcpp::Node{
 public:
  explicit Simulator(const std::string & name);
  virtual ~Simulator();
  bool initializeSimulation();
  void runSimulation();

  //// callbacks
  void onPauseSimulation(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void onUnpauseSimulation(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void spawnCallback();

 protected:
  //void reconfigureCB(SimConfig& config, uint32_t level);
  //dynamic_reconfigure::Server<SimConfig> server_;

 private:
  void updateRobotPositionFromTF();
  void publishAgents();
  void publishGroups();
  void publishObstacles();
  void publishRobotPosition();

  bool paused_;
  rclcpp::TimerBase::SharedPtr spawn_timer_;

  // publishers
  rclcpp::Publisher<LineObstacles>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<AgentStates>::SharedPtr pub_agent_states_;
  rclcpp::Publisher<AgentGroups>::SharedPtr pub_agent_groups_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robot_position_;

  // provided services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_pause_simulation_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_unpause_simulation_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Agent* robot_;

  geometry_msgs::msg::TransformStamped last_robot_pose_;
  geometry_msgs::msg::Quaternion last_robot_orientation_;
  float robot_radius_, agent_radius_, force_factor_social_;
  inline std::string agentStateToActivity(
    const AgentStateMachine::AgentState& state) const;

  inline std_msgs::msg::Header createMsgHeader() const;

};

#endif

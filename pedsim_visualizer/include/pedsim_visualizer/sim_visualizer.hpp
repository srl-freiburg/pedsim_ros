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

#ifndef SIM_VISUALIZER_H
#define SIM_VISUALIZER_H

#include <functional>
#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"

#include <pedsim_msgs/msg/agent_force.hpp>
#include <pedsim_msgs/msg/agent_group.hpp>
#include <pedsim_msgs/msg/agent_groups.hpp>
#include <pedsim_msgs/msg/agent_state.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/line_obstacle.hpp>
#include <pedsim_msgs/msg/line_obstacles.hpp>
#include <pedsim_msgs/msg/social_activities.hpp>
#include <pedsim_msgs/msg/social_activity.hpp>
#include <pedsim_msgs/msg/social_relation.hpp>
#include <pedsim_msgs/msg/social_relations.hpp>

#include <spencer_tracking_msgs/msg/tracked_person.hpp>
#include <spencer_tracking_msgs/msg/tracked_persons.hpp>
#include <spencer_tracking_msgs/msg/tracked_group.hpp>
#include <spencer_tracking_msgs/msg/tracked_groups.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pedsim_utils/geometry.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <pedsim_visualizer/PedsimVisualizerConfig.h>

namespace pedsim {

class SimVisualizer : public rclcpp::Node
{
 public:
  //using VizConfig = pedsim_visualizer::PedsimVisualizerConfig;

  explicit SimVisualizer(const std::string & name);
  ////SimVisualizer(const SimVisualizer& other) = delete;

  void run();

  // callbacks.
  void agentStatesCallBack(const pedsim_msgs::msg::AgentStates::SharedPtr agents);
  void agentGroupsCallBack(const pedsim_msgs::msg::AgentGroups::SharedPtr groups);
  void obstaclesCallBack(const pedsim_msgs::msg::LineObstacles::SharedPtr obstacles);

 protected:
  /// publishers
  void publishAgentVisuals();
  void publishGroupVisuals();
  void publishObstacleVisuals();

 private:
  void setupPublishersAndSubscribers();

  /// publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_obstacles_visuals_;
  rclcpp::Publisher<spencer_tracking_msgs::msg::TrackedPersons>::SharedPtr pub_person_visuals_;
  rclcpp::Publisher<spencer_tracking_msgs::msg::TrackedGroups>::SharedPtr pub_group_visuals_;

  /// Subscribers.
  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr sub_states_;
  rclcpp::Subscription<pedsim_msgs::msg::AgentGroups>::SharedPtr sub_groups_;
  rclcpp::Subscription<pedsim_msgs::msg::LineObstacles>::SharedPtr sub_obstacles_;
  /// Local data queues.
  std::queue<pedsim_msgs::msg::AgentStates::SharedPtr> q_people_;
  std::queue<pedsim_msgs::msg::AgentGroups::SharedPtr> q_groups_;
  std::queue<pedsim_msgs::msg::LineObstacles::SharedPtr> q_obstacles_;

  std::string frame_id_;
  bool walls_published_;
  float walls_resolution_;
};
}  // namespace pedsim

#endif

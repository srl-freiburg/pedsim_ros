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

#include <ros/console.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <functional>
#include <memory>
#include <queue>

#include <pedsim_msgs/AgentForce.h>
#include <pedsim_msgs/AgentGroup.h>
#include <pedsim_msgs/AgentGroups.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/LineObstacle.h>
#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/Waypoint.h>
#include <pedsim_msgs/Waypoints.h>

#include <pedsim_msgs/SocialActivities.h>
#include <pedsim_msgs/SocialActivity.h>
#include <pedsim_msgs/SocialRelation.h>
#include <pedsim_msgs/SocialRelations.h>
#include <pedsim_msgs/TrackedGroup.h>
#include <pedsim_msgs/TrackedGroups.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedPersons.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <pedsim_visualizer/PedsimVisualizerConfig.h>

namespace pedsim {

class SimVisualizer {
 public:
  using VizConfig = pedsim_visualizer::PedsimVisualizerConfig;

  explicit SimVisualizer(const ros::NodeHandle& node_in);
  ~SimVisualizer();
  SimVisualizer(const SimVisualizer& other) = delete;

  void run();

  // callbacks.
  void agentStatesCallBack(const pedsim_msgs::AgentStatesConstPtr& agents);
  void agentGroupsCallBack(const pedsim_msgs::AgentGroupsConstPtr& groups);
  void obstaclesCallBack(const pedsim_msgs::LineObstaclesConstPtr& obstacles);
  void waypointsCallBack(const pedsim_msgs::WaypointsConstPtr& waypoints);

 protected:
  /// publishers
  void publishAgentVisuals();
  void publishRelationVisuals();
  void publishActivityVisuals();
  void publishGroupVisuals();
  void publishObstacleVisuals();
  void publishWaypointVisuals();

 private:
  void setupPublishersAndSubscribers();

  ros::NodeHandle nh_;
  double hz_;

  /// publishers
  ros::Publisher pub_obstacles_visuals_;
  ros::Publisher pub_person_visuals_;
  ros::Publisher pub_group_visuals_;
  ros::Publisher pub_forces_;
  ros::Publisher pub_waypoints_;

  /// Subscribers.
  ros::Subscriber sub_states_;
  ros::Subscriber sub_groups_;
  ros::Subscriber sub_obstacles_;
  ros::Subscriber sub_waypoints_;

  /// Local data queues.
  std::queue<pedsim_msgs::AgentStatesConstPtr> q_people_;
  std::queue<pedsim_msgs::AgentGroupsConstPtr> q_groups_;
  std::queue<pedsim_msgs::LineObstaclesConstPtr> q_obstacles_;
  std::queue<pedsim_msgs::WaypointsConstPtr> q_waypoints_;
};
}  // namespace pedsim

#endif

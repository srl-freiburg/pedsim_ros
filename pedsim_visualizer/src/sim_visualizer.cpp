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

#include <pedsim_visualizer/sim_visualizer.h>

#include <pedsim_utils/geometry.h>

namespace pedsim {

SimVisualizer::SimVisualizer(const ros::NodeHandle& node_in) : nh_{node_in} {
  setupPublishersAndSubscribers();
}
SimVisualizer::~SimVisualizer() {
  pub_obstacles_visuals_.shutdown();
  pub_person_visuals_.shutdown();
  pub_group_visuals_.shutdown();

  /// Subscribers.
  sub_states_.shutdown();
  sub_groups_.shutdown();
  sub_obstacles_.shutdown();
}

void SimVisualizer::run() {
  ros::Rate r(25.);

  while (ros::ok()) {
    publishAgentVisuals();
    publishGroupVisuals();
    publishObstacleVisuals();

    ros::spinOnce();
    r.sleep();
  }
}

// callbacks.
void SimVisualizer::agentStatesCallBack(
    const pedsim_msgs::AgentStatesConstPtr& agents) {
  q_people_.emplace(agents);
}
void SimVisualizer::agentGroupsCallBack(
    const pedsim_msgs::AgentGroupsConstPtr& groups) {
  q_groups_.emplace(groups);
}

void SimVisualizer::obstaclesCallBack(
    const pedsim_msgs::LineObstaclesConstPtr& obstacles) {
  q_obstacles_.emplace(obstacles);
}

/// publishers
void SimVisualizer::publishAgentVisuals() {
  if (q_people_.size() < 1) {
    return;
  }

  const auto current_states = q_people_.front();

  pedsim_msgs::TrackedPersons tracked_people;
  tracked_people.header = current_states->header;

  for (const auto& agent_state : current_states->agent_states) {
    if (agent_state.type == 2) continue;

    pedsim_msgs::TrackedPerson person;
    person.track_id = agent_state.id;
    person.is_occluded = false;
    person.detection_id = agent_state.id;

    const double theta =
        std::atan2(agent_state.twist.linear.y, agent_state.twist.linear.x);
    geometry_msgs::PoseWithCovariance pose_with_cov;
    pose_with_cov.pose.position.x = agent_state.pose.position.x;
    pose_with_cov.pose.position.y = agent_state.pose.position.y;
    pose_with_cov.pose.position.z = agent_state.pose.position.z;
    pose_with_cov.pose.orientation = angleToQuaternion(theta);
    person.pose = pose_with_cov;

    geometry_msgs::TwistWithCovariance twist_with_cov;
    twist_with_cov.twist.linear.x = agent_state.twist.linear.x;
    twist_with_cov.twist.linear.y = agent_state.twist.linear.y;
    person.twist = twist_with_cov;

    tracked_people.tracks.push_back(person);
  }

  pub_person_visuals_.publish(tracked_people);
  q_people_.pop();
}

void SimVisualizer::publishGroupVisuals() {
  if (q_groups_.empty()) {
    ROS_DEBUG_STREAM("Skipping publishing groups");
    return;
  }

  const auto sim_groups = q_groups_.front();

  pedsim_msgs::TrackedGroups tracked_groups;
  tracked_groups.header = sim_groups->header;

  for (const auto& ag : sim_groups->groups) {
    pedsim_msgs::TrackedGroup group;
    group.group_id = ag.group_id;

    // TODO - update.
    // group.age = ag.age;
    group.centerOfGravity.pose = ag.center_of_mass;
    std::copy(ag.members.begin(), ag.members.end(),
              std::back_inserter(group.track_ids));
    tracked_groups.groups.emplace_back(group);
  }

  pub_group_visuals_.publish(tracked_groups);
  q_groups_.pop();
}

void SimVisualizer::publishObstacleVisuals() {
  if (q_obstacles_.size() < 1) {
    return;
  }

  const auto current_obstacles = q_obstacles_.front();

  visualization_msgs::Marker walls_marker;
  walls_marker.header.frame_id = "odom";
  walls_marker.header.stamp = ros::Time();
  walls_marker.id = 10000;
  walls_marker.color.a = 1.0;
  walls_marker.color.r = 0.647059;
  walls_marker.color.g = 0.164706;
  walls_marker.color.b = 0.164706;
  walls_marker.scale.x = 1.0;
  walls_marker.scale.y = 1.0;
  walls_marker.scale.z = 2.0;
  walls_marker.pose.position.z = walls_marker.scale.z / 2.0;
  walls_marker.type = visualization_msgs::Marker::CUBE_LIST;

  for (const auto& line : current_obstacles->obstacles) {
    for (const auto& cell : LineObstacleToCells(line.start.x, line.start.y,
                                                line.end.x, line.end.y)) {
      geometry_msgs::Point p;
      p.x = cell.first;
      p.y = cell.second;
      p.z = 0.0;
      walls_marker.points.push_back(p);
    }
  }

  pub_obstacles_visuals_.publish(walls_marker);
}

void SimVisualizer::setupPublishersAndSubscribers() {
  pub_obstacles_visuals_ =
      nh_.advertise<visualization_msgs::Marker>("walls", 1, true);
  pub_person_visuals_ =
      nh_.advertise<pedsim_msgs::TrackedPersons>("tracked_persons", 1);
  pub_group_visuals_ =
      nh_.advertise<pedsim_msgs::TrackedGroups>("tracked_groups", 1);

  // TODO - get simulator node name by param.
  sub_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1,
                              &SimVisualizer::agentStatesCallBack, this);
  sub_obstacles_ = nh_.subscribe("/pedsim_simulator/simulated_walls", 1,
                                 &SimVisualizer::obstaclesCallBack, this);
  sub_groups_ = nh_.subscribe("/pedsim_simulator/simulated_groups", 1,
                              &SimVisualizer::agentGroupsCallBack, this);
}

}  // namespace pedsim

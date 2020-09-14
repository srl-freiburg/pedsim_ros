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

const static double DEFAULT_VIZ_HZ = 25.0;

SimVisualizer::SimVisualizer(const ros::NodeHandle& node_in) : nh_{node_in} {
  setupPublishersAndSubscribers();
  nh_.param<double>("hz", hz_, DEFAULT_VIZ_HZ);
  if (hz_ < 0) {
    hz_ = DEFAULT_VIZ_HZ;
  }
}
SimVisualizer::~SimVisualizer() {
  pub_obstacles_visuals_.shutdown();
  pub_person_visuals_.shutdown();
  pub_group_visuals_.shutdown();

  /// Subscribers.
  sub_states_.shutdown();
  sub_groups_.shutdown();
  sub_obstacles_.shutdown();
  sub_waypoints_.shutdown();
}

void SimVisualizer::run() {
  ros::Rate r(hz_);

  while (ros::ok()) {
    publishAgentVisuals();
    publishGroupVisuals();
    publishObstacleVisuals();
    publishWaypointVisuals();

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

void SimVisualizer::waypointsCallBack(
    const pedsim_msgs::WaypointsConstPtr& waypoints) {
  q_waypoints_.emplace(waypoints);
}

/// publishers
void SimVisualizer::publishAgentVisuals() {
  if (q_people_.size() < 1) {
    return;
  }

  const auto current_states = q_people_.front();

  visualization_msgs::MarkerArray forces_markers;
  visualization_msgs::Marker force_marker;
  force_marker.header = current_states->header;
  force_marker.type = visualization_msgs::Marker::ARROW;
  force_marker.action = visualization_msgs::Marker::ADD;
  force_marker.lifetime = ros::Duration(1.0 / hz_);
  force_marker.scale.x = 0.05; // shaft diameter
  force_marker.scale.y = 0.1; // head diameter
  force_marker.scale.z = 0.3; // head length
  force_marker.color.a = 1.0;
  force_marker.pose.orientation.w = 1.0;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;


  pedsim_msgs::TrackedPersons tracked_people;
  tracked_people.header = current_states->header;

  for (const auto& agent_state : current_states->agent_states) {

    if (agent_state.type == 2) continue;

    force_marker.ns = "agent_state_" + std::to_string(agent_state.id);
    force_marker.points.clear();
    force_marker.color.r = 0.0;
    force_marker.color.g = 0.0;
    force_marker.color.b = 0.0;

    p1.x = agent_state.pose.position.x;
    p1.y = agent_state.pose.position.y;
    p1.z = agent_state.pose.position.z;
    force_marker.points.push_back(p1);

    // desired_force
    force_marker.id = 0;
    p2.x = p1.x + agent_state.forces.desired_force.x;
    p2.y = p1.y + agent_state.forces.desired_force.y;
    p2.z = p1.z + agent_state.forces.desired_force.z;
    force_marker.color.r = 1.0;
    force_marker.points.push_back(p2);
    forces_markers.markers.push_back(force_marker);

    // obstacle_force
    force_marker.id = 1;
    force_marker.points[1].x = p1.x + agent_state.forces.obstacle_force.x;
    force_marker.points[1].y = p1.y + agent_state.forces.obstacle_force.y;
    force_marker.points[1].z = p1.z + agent_state.forces.obstacle_force.z;
    force_marker.color.r = 0.0;
    force_marker.color.g = 1.0;
    forces_markers.markers.push_back(force_marker);

    // social_force
    force_marker.id = 2;
    force_marker.points[1].x = p1.x + agent_state.forces.social_force.x;
    force_marker.points[1].y = p1.y + agent_state.forces.social_force.y;
    force_marker.points[1].z = p1.z + agent_state.forces.social_force.z;
    force_marker.color.r = 0.0;
    force_marker.color.g = 0.0;
    force_marker.color.b = 1.0;
    forces_markers.markers.push_back(force_marker);

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
  pub_forces_.publish(forces_markers);
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
  walls_marker.header = current_obstacles->header;
  walls_marker.id = 10000;
  walls_marker.color.a = 1.0;
  walls_marker.color.r = 0.647059;
  walls_marker.color.g = 0.164706;
  walls_marker.color.b = 0.164706;
  walls_marker.scale.x = 1.0;
  walls_marker.scale.y = 1.0;
  walls_marker.scale.z = 2.0;
  walls_marker.pose.position.z = walls_marker.scale.z / 2.0;
  walls_marker.pose.orientation.w = 1.0;
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
  q_obstacles_.pop();
}

void SimVisualizer::publishWaypointVisuals() {
  if (q_waypoints_.size() < 1) {
    return;
  }

  const auto current_waypoints = q_waypoints_.front();
  visualization_msgs::Marker wp_marker;
  wp_marker.header = current_waypoints->header;
  wp_marker.action = visualization_msgs::Marker::ADD;
  wp_marker.pose.orientation.w = 1.0;
  wp_marker.color.a = 1.0;
  std::string text;

  visualization_msgs::MarkerArray waypoint_markers;
  for (const auto& waypoint : current_waypoints->waypoints) {
    text = waypoint.name;

    wp_marker.ns = text;

    switch (waypoint.behavior) {
      case pedsim_msgs::Waypoint::BHV_SIMPLE: {
        wp_marker.color.r = 1.0;
        wp_marker.color.g = 0.95;
        wp_marker.color.b = 0.7;
        break;
      }
      case pedsim_msgs::Waypoint::BHV_SOURCE: {
        wp_marker.color.r = 0.0;
        wp_marker.color.g = 1.0;
        wp_marker.color.b = 1.0;
        text += "(source)";
        break;
      }
      case pedsim_msgs::Waypoint::BHV_SINK: {
        wp_marker.color.r = 1.0;
        wp_marker.color.g = 0.37;
        wp_marker.color.b = 0.07;
        text += "(sink)";
        break;
      }
    }

    wp_marker.id = 0;
    wp_marker.type = visualization_msgs::Marker::CUBE;
    wp_marker.scale.x = 0.2;
    wp_marker.scale.y = 0.2;
    wp_marker.scale.z = 0.2;
    wp_marker.pose.position.x = waypoint.position.x;
    wp_marker.pose.position.y = waypoint.position.y;
    wp_marker.pose.position.z = 0.1;
    waypoint_markers.markers.push_back(wp_marker);

    wp_marker.id = 1;
    wp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    wp_marker.scale.x = 0.0;
    wp_marker.scale.y = 0.0;
    wp_marker.text = text;
    wp_marker.pose.position.z = 0.35;
    waypoint_markers.markers.push_back(wp_marker);

    wp_marker.id = 2;
    wp_marker.text = "";
    wp_marker.type = visualization_msgs::Marker::CYLINDER;
    wp_marker.scale.x = waypoint.radius;
    wp_marker.scale.y = waypoint.radius;
    wp_marker.scale.z = 0.01;
    wp_marker.pose.position.z = 0.005;
    waypoint_markers.markers.push_back(wp_marker);
  }
  pub_waypoints_.publish(waypoint_markers);

  q_waypoints_.pop();
}

void SimVisualizer::setupPublishersAndSubscribers() {
  pub_obstacles_visuals_ =
      nh_.advertise<visualization_msgs::Marker>("walls", 1, true);
  pub_person_visuals_ =
      nh_.advertise<pedsim_msgs::TrackedPersons>("tracked_persons", 1);
  pub_group_visuals_ =
      nh_.advertise<pedsim_msgs::TrackedGroups>("tracked_groups", 1);
  pub_forces_ =
    nh_.advertise<visualization_msgs::MarkerArray>("forces", 1);
  pub_waypoints_ =
    nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1);

  // TODO - get simulator node name by param.
  sub_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1,
                              &SimVisualizer::agentStatesCallBack, this);
  sub_obstacles_ = nh_.subscribe("/pedsim_simulator/simulated_walls", 1,
                                 &SimVisualizer::obstaclesCallBack, this);
  sub_groups_ = nh_.subscribe("/pedsim_simulator/simulated_groups", 1,
                              &SimVisualizer::agentGroupsCallBack, this);
  sub_waypoints_ = nh_.subscribe("/pedsim_simulator/simulated_waypoints", 1,
                              &SimVisualizer::waypointsCallBack, this);
}

}  // namespace pedsim

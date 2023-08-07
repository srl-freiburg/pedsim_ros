// Copyright 2019 Open Source Robotics Foundation, Inc.
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

/*
 * \brief  Change actor position based on pedsim output
 *
 * \author  mahmoud
 * \mantainer jginesclavero
 *
 * \date  6 March 2020
 */
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <pedsim_msgs/msg/tracked_person.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins {
class ActorPosesPlugin : public gazebo::ModelPlugin {
public:
  ActorPosesPlugin() {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model_ = _model;
    world_ = _model->GetWorld();
    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    pedsim_sub_ = ros_node_->create_subscription<pedsim_msgs::msg::AgentStates>(
        "pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
        std::bind(&ActorPosesPlugin::OnAgentStates, this,
                  std::placeholders::_1));
  }

  // call back function when receive rosmsg
  void OnAgentStates(const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
    std::string model_name;
    for (unsigned int mdl = 0; mdl < world_->ModelCount(); mdl++) {
      gazebo::physics::ModelPtr tmp_model = world_->ModelByIndex(mdl);
      std::string frame_id = tmp_model->GetName();
      for (uint actor = 0; actor < msg->agent_states.size(); actor++) {
        if (frame_id == std::to_string(msg->agent_states[actor].id)) {
          if (std::isnan(msg->agent_states[actor].twist.linear.x) ||
              std::isnan(msg->agent_states[actor].twist.linear.y) ||
              std::isnan(msg->agent_states[actor].pose.position.x) ||
              std::isnan(msg->agent_states[actor].pose.position.y) ||
              std::isnan(msg->agent_states[actor].pose.position.z)) {
            continue;
          }
          const double theta =
              std::atan2(msg->agent_states[actor].twist.linear.y,
                         msg->agent_states[actor].twist.linear.x);

          RCLCPP_DEBUG(ros_node_->get_logger(), "actor_id: %s",
                       std::to_string(msg->agent_states[actor].id).c_str());

          ignition::math::Pose3d gzb_pose;
          tf2::Quaternion qt;
          qt.setRPY(0.0, 0.0, theta + (M_PI / 2));
          qt.normalize();

          gzb_pose.Pos().Set(msg->agent_states[actor].pose.position.x,
                             msg->agent_states[actor].pose.position.y,
                             msg->agent_states[actor].pose.position.z);
          gzb_pose.Rot().Set(qt.w(), qt.x(), qt.y(), qt.z());
          try {
            tmp_model->SetWorldPose(gzb_pose);
          } catch (gazebo::common::Exception gz_ex) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Error setting pose %s - %s",
                         frame_id.c_str(), gz_ex.GetErrorStr().c_str());
          }
        }
      }
    }
  }

private:
  /// Pointer to world.
  gazebo::physics::WorldPtr world_;
  /// Pointer to model.
  gazebo::physics::ModelPtr model_;
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Subscriber to pedsim output
  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr pedsim_sub_;
};
GZ_REGISTER_MODEL_PLUGIN(ActorPosesPlugin)
} // namespace gazebo_plugins

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
#include "pedsim_tf2/pedsim_tf2_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

namespace pedsim {
PedsimTF2::PedsimTF2(const std::string &name) : Node(name), states_() {
  sub_ = create_subscription<pedsim_msgs::msg::AgentStates>(
      "pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
      std::bind(&PedsimTF2::agentsCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void PedsimTF2::agentsCallback(
    const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
  states_ = msg;
}

bool PedsimTF2::getTFfromAgent(pedsim_msgs::msg::AgentState actor,
                               TransformStamped &tf) {
  double theta = std::atan2(actor.twist.linear.y, actor.twist.linear.x);
  tf2::Quaternion qt;
  qt.setRPY(0.0, 0.0, theta);
  qt.normalize();
  tf.transform.translation.x = actor.pose.position.x;
  tf.transform.translation.y = actor.pose.position.y;
  tf.transform.translation.z = actor.pose.position.z;
  tf.transform.rotation = tf2::toMsg(qt);
  if (std::isnan(theta) || std::isnan(tf.transform.translation.x) ||
      std::isnan(tf.transform.translation.y) ||
      std::isnan(tf.transform.translation.z)) {
    return false;
  }
  return true;
}

void PedsimTF2::step() {
  rclcpp::Rate loop_rate(50ms);
  while (rclcpp::ok()) {
    if (states_ != NULL) {
      for (auto actor : states_->agent_states) {
        TransformStamped agent_tf;
        if (getTFfromAgent(actor, agent_tf)) {
          agent_tf.header.frame_id = "map";
          agent_tf.header.stamp = now();
          agent_tf.child_frame_id = "agent_" + std::to_string(actor.id);
          tf_broadcaster_->sendTransform(agent_tf);
        }
      }
    }
    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
  }
}

}; // namespace pedsim

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto pedsim_tf2 = std::make_shared<pedsim::PedsimTF2>("pedsim_tf2_node");
  pedsim_tf2->step();
  rclcpp::shutdown();

  return 0;
}
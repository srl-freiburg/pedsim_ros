/**
* Copyright 2014- Social Robotics Lab, University of Freiburg
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

#include <pedsim_sensors/people_point_cloud.h>

#include <random>

namespace pedsim_ros {

PeoplePointCloud::PeoplePointCloud(const ros::NodeHandle& node_handle,
                                   const double rate, const FoVPtr& fov)
    : PedsimSensor(node_handle, rate, fov) {
  pub_signals_local_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_local", 1);
  pub_signals_global_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_global", 1);

  sub_simulated_agents_ =
      nh_.subscribe("/pedsim_simulator/simulated_agents", 1,
                    &PeoplePointCloud::agentStatesCallBack, this);
}

void PeoplePointCloud::broadcast() {
  if (q_agents_.size() < 1) {
    return;
  }

  constexpr int point_density = 100;
  const auto people_signal = q_agents_.front();
  const int num_points = people_signal->agent_states.size() * point_density;

  std::default_random_engine generator;

  // \todo - Read params from config file.
  std::uniform_int_distribution<int> color_distribution(1, 255);
  std::uniform_real_distribution<float> height_distribution(0, 1.8);
  std::uniform_real_distribution<float> width_distribution(-0.18, 0.18);

  sensor_msgs::PointCloud pcd_global;
  pcd_global.header.stamp = ros::Time::now();
  pcd_global.header.frame_id = people_signal->header.frame_id;
  pcd_global.points.resize(num_points);
  pcd_global.channels.resize(1);
  pcd_global.channels[0].name = "intensities";
  pcd_global.channels[0].values.resize(num_points);

  sensor_msgs::PointCloud pcd_local;
  pcd_local.header.stamp = ros::Time::now();
  pcd_local.header.frame_id = robot_odom_.header.frame_id;
  pcd_local.points.resize(num_points);
  pcd_local.channels.resize(1);
  pcd_local.channels[0].name = "intensities";
  pcd_local.channels[0].values.resize(num_points);

  // prepare the transform to robot odom frame.
  tf::StampedTransform robot_transform;
  try {
    transform_listener_->lookupTransform(robot_odom_.header.frame_id,
                                         people_signal->header.frame_id,
                                         ros::Time(0), robot_transform);
  } catch (tf::TransformException& e) {
    ROS_WARN_STREAM_THROTTLE(5.0, "TFP lookup from ["
                                      << people_signal->header.frame_id
                                      << "] to [" << robot_odom_.header.frame_id
                                      << "] failed. Reason: " << e.what());
    return;
  }

  size_t index = 0;
  for (const auto& person : people_signal->agent_states) {
    const int person_pcd_color = color_distribution(generator);
    for (size_t j = 0; j < point_density; ++j) {
      if (fov_->inside(person.pose.position.x, person.pose.position.y)) {
        // Frame transformations.
        // - Make sure person is in the same frame as robot
        const tf::Vector3 point(
            person.pose.position.x + width_distribution(generator),
            person.pose.position.y + width_distribution(generator), 0.);
        const auto transformed_point = transformPoint(robot_transform, point);

        pcd_local.points[index].x = transformed_point.getOrigin().x();
        pcd_local.points[index].y = transformed_point.getOrigin().y();
        pcd_local.points[index].z = height_distribution(generator);
        pcd_local.channels[0].values[index] = person_pcd_color;

        // Global observations
        pcd_global.points[index].x =
            person.pose.position.x + width_distribution(generator);
        pcd_global.points[index].y =
            person.pose.position.y + width_distribution(generator);
        pcd_global.points[index].z = height_distribution(generator);
        pcd_global.channels[0].values[index] = person_pcd_color;
      }

      index++;
    }
  }

  if (pcd_local.channels[0].values.size() > 1) {
    pub_signals_local_.publish(pcd_local);
  }
  if (pcd_global.channels[0].values.size() > 1) {
    pub_signals_global_.publish(pcd_global);
  }

  q_agents_.pop();
};

void PeoplePointCloud::run() {
  ros::Rate r(rate_);

  while (ros::ok()) {
    broadcast();

    ros::spinOnce();
    r.sleep();
  }
}

void PeoplePointCloud::agentStatesCallBack(
    const pedsim_msgs::AgentStatesConstPtr& agents) {
  q_agents_.emplace(agents);
}

}  // namespace

// --------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "pedsim_people_sensor");
  ros::NodeHandle node("~");

  double init_x = 0.0, init_y = 0.0, fov_range = 0.0;
  node.param<double>("pose_initial_x", init_x, 0.0);
  node.param<double>("pose_initial_y", init_y, 0.0);
  node.param<double>("fov_range", fov_range, 15.);

  pedsim_ros::FoVPtr circle_fov;
  circle_fov.reset(new pedsim_ros::CircularFov(init_x, init_y, fov_range));

  double sensor_rate = 0.0;
  node.param<double>("rate", sensor_rate, 25.0);

  pedsim_ros::PeoplePointCloud pcd_sensor(node, sensor_rate, circle_fov);
  ROS_INFO_STREAM("Initialized people PCD sensor with center: (" 
    << init_x << ", " << init_y << ") and range: " << fov_range);

  pcd_sensor.run();
  return 0;
}

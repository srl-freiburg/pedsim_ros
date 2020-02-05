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

#include <pedsim_sensors/occlusion_point_cloud.h>
#include <pedsim_utils/geometry.h>
#include <math.h>
#include <random>
#define INF 100000000

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

namespace pedsim_ros {

using Cell = std::complex<float>;

PointCloud::PointCloud(const ros::NodeHandle& node_handle,
                            const double rate, const int resol, const FoVPtr& fov)
    : PedsimSensor(node_handle, rate, fov) {
  resol_ = resol;
  pub_signals_local_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_local", 1);
  pub_signals_global_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_global", 1);

  sub_simulated_obstacles_ =
      nh_.subscribe("/pedsim_simulator/simulated_walls", 1,
                    &PointCloud::obstaclesCallBack, this);
  sub_simulated_agents_ =
      nh_.subscribe("/pedsim_simulator/simulated_agents", 1,
                    &PointCloud::agentStatesCallBack, this);
}

uint PointCloud::rad_to_index(float rad_){
  auto rad = atan2(sin(rad_), cos(rad_));
  return (rad + M_PI) * resol_ / (2 * M_PI);
}

float PointCloud::index_to_rad(uint index){
  return -M_PI + 2 * M_PI / resol_ * index;
}

uint PointCloud::fit_index(int index) {
  if (index < 0) {
    return index % resol_ + resol_;
  }
  else{
    return index % resol_; 
  }
}

void PointCloud::fillDetectedObss(std::vector<Cell>& detected_obss, Cell cell, float width){
  Cell obs = cell - Cell(fov_->origin_x, fov_->origin_y);
  float edge_x = -width * sgn(obs.real());
  float edge_y = -width * sgn(obs.imag());
  for (float dw = -width; dw <= width; dw += width / 10) {
    for (auto new_obs : {obs + Cell(edge_x, dw), obs + Cell(dw, edge_y)}){
      auto r = std::abs(new_obs);
      auto theta = std::arg(new_obs);
      uint rad_index = rad_to_index(theta);
      // fill adj indexes with same values
      for (int i : {-2, -1, 0, 1, 2}){
        uint index = fit_index(rad_index + i);
        if (r < std::abs(detected_obss[index])) {
          auto polar = std::polar(r, theta);
          detected_obss[index] = polar;
        }
      }
    }
  }
}

void PointCloud::broadcast() {

  std::vector<Cell> detected_obss(resol_, Cell(INF, INF));

  // obstacles 
  if (q_obstacles_.size() < 1 || q_agents_.size() < 1) {
    return;
  }

  // fill by cells
  std::vector<std::pair<float, float>> all_cells;
  const auto sim_obstacles = q_obstacles_.front();
  for (const auto& line : sim_obstacles->obstacles) {
    const auto cells = pedsim::LineObstacleToCells(line.start.x, line.start.y,
                                                   line.end.x, line.end.y);
    std::copy(cells.begin(), cells.end(), std::back_inserter(all_cells));
  }
  for (const auto& pair_cell : all_cells) {
    // convert pair to complex
    auto cell = Cell(pair_cell.first, pair_cell.second);
    fillDetectedObss(detected_obss, cell, obs_width);
  }

  // fill by agents
  const auto people_signal = q_agents_.front();
  for (const auto& person : people_signal->agent_states) {
    Cell cell(person.pose.position.x, person.pose.position.y);
    fillDetectedObss(detected_obss, cell, human_width);
  }

  constexpr int point_density = 10;
  const int num_points = detected_obss.size() * point_density;

  std::default_random_engine generator;

  // \todo - Read params from config file.
  std::uniform_int_distribution<int> color_distribution(1, 255);
  std::uniform_real_distribution<float> height_distribution(0, 1);
  std::uniform_real_distribution<float> width_distribution(-0.05, 0.05);

  sensor_msgs::PointCloud pcd_global;
  pcd_global.header.stamp = ros::Time::now();
  pcd_global.header.frame_id = sim_obstacles->header.frame_id;
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
                                         sim_obstacles->header.frame_id,
                                         ros::Time(0), robot_transform);
  } catch (tf::TransformException& e) {
    ROS_WARN_STREAM_THROTTLE(5.0, "TFP lookup from ["
                                      << sim_obstacles->header.frame_id
                                      << "] to [" << robot_odom_.header.frame_id
                                      << "] failed. Reason: " << e.what());
    return;
  }

  size_t index = 0;
  for (const auto& obs : detected_obss) {
    Cell cell = obs + Cell(fov_->origin_x, fov_->origin_y);
    const int cell_color = color_distribution(generator);

    for (size_t j = 0; j < point_density; ++j) {
      if (fov_->inside(cell.real(), cell.imag())) {
        const tf::Vector3 point(cell.real() + width_distribution(generator),
                                cell.imag() + width_distribution(generator),
                                0.);
        const auto transformed_point = transformPoint(robot_transform, point);

        pcd_local.points[index].x = transformed_point.getOrigin().x();
        pcd_local.points[index].y = transformed_point.getOrigin().y();
        pcd_local.points[index].z = height_distribution(generator);
        pcd_local.channels[0].values[index] = cell_color;

        // Global observations.
        pcd_global.points[index].x = cell.real() + width_distribution(generator);
        pcd_global.points[index].y =
            cell.imag() + width_distribution(generator);
        pcd_global.points[index].z = height_distribution(generator);
        pcd_global.channels[0].values[index] = cell_color;
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

  q_obstacles_.pop();
  q_agents_.pop();
};

void PointCloud::run() {
  ros::Rate r(rate_);

  while (ros::ok()) {
    broadcast();

    ros::spinOnce();
    r.sleep();
  }
}

void PointCloud::obstaclesCallBack(
    const pedsim_msgs::LineObstaclesConstPtr& obstacles) {
  q_obstacles_.emplace(obstacles);
}

void PointCloud::agentStatesCallBack(
    const pedsim_msgs::AgentStatesConstPtr& agents) {
  q_agents_.emplace(agents);
}

}  // namespace pedsim_ros

// --------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "pedsim_occlusion_sensor");
  ros::NodeHandle node("~");

  double init_x = 0.0, init_y = 0.0, fov_range = 0.0;
  node.param<double>("pose_initial_x", init_x, 0.0);
  node.param<double>("pose_initial_y", init_y, 0.0);
  node.param<double>("fov_range", fov_range, 15.);

  pedsim_ros::FoVPtr circle_fov;
  circle_fov.reset(new pedsim_ros::CircularFov(init_x, init_y, fov_range));

  double sensor_rate = 0.0;
  int sensor_resol = 360;
  node.param<double>("rate", sensor_rate, 25.0);
  node.param<int>("resol", sensor_resol, 360);

  pedsim_ros::PointCloud pcd_sensor(node, sensor_rate, sensor_resol, circle_fov);
  ROS_INFO_STREAM("Initialized occlusion PCD sensor with center: ("
                  << init_x << ", " << init_y << ") , range: " << fov_range << ", resolution: " << sensor_resol);

  pcd_sensor.run();
  return 0;
}

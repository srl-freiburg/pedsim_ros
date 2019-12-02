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

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <pedsim_sensors/pedsim_sensor.h>

#include <queue>

#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/AgentStates.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace pedsim_ros {

class PointCloud : public PedsimSensor {
 public:
  PointCloud(const ros::NodeHandle& node_handle, const double rate, const int resol, 
                     const FoVPtr& fov);
  virtual ~PointCloud() = default;

  void broadcast() override;
  void run();
  void obstaclesCallBack(const pedsim_msgs::LineObstaclesConstPtr& obstacles);
  void agentStatesCallBack(const pedsim_msgs::AgentStatesConstPtr& agents);

  // detected obss is a 360 deg scan map
  uint rad_to_index(float rad);
  float index_to_rad(uint index);
  uint fit_index(int index);
  void fillDetectedObss(std::vector<std::complex<float>>& detected_obss,
                        std::complex<float> obs, float width);

 private:
  ros::Subscriber sub_simulated_obstacles_;
  ros::Subscriber sub_simulated_agents_;

  std::queue<pedsim_msgs::LineObstaclesConstPtr> q_obstacles_;
  std::queue<pedsim_msgs::AgentStatesConstPtr> q_agents_;

 protected:
  int resol_;
  float human_width = 0.2;
  float obs_width = 0.5;
};

}  // namespace pedsim_ros

#endif

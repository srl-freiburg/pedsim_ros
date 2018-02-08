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

#ifndef PEDSIM_SENSOR_H
#define PEDSIM_SENSOR_H

#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

namespace pedsim_ros {

struct FoV {
  double origin_x;
  double origin_y;

  FoV() = default;
  FoV(const double x, const double y) : origin_x{x}, origin_y{y} {}
  virtual bool inside(const double x, const double y) const = 0;
  virtual void updateViewpoint(const double new_x, const double new_y) = 0;
};

using FoVPtr = std::shared_ptr<FoV>;

struct CircularFov : FoV {
  double radius;

  CircularFov(const double cx, const double cy, const double r)
      : FoV(cx, cy), radius{r} {}

  bool inside(const double x, const double y) const override {
    return std::hypot(x - origin_x, y - origin_y) < radius;
  }
  void updateViewpoint(const double new_x, const double new_y) override {
    origin_x = new_x;
    origin_y = new_y;
  }
};

/// \brief A sensor interface.
class PedsimSensor {
 public:
  PedsimSensor(const ros::NodeHandle& node_handle, const double rate,
               const FoVPtr& fov)
      : nh_{node_handle}, rate_{rate} {
    if (fov == nullptr) {
      ROS_FATAL_STREAM("Sensor FoV cannot be null.");
    }
    fov_ = fov;
    // Set up robot odometry subscriber.
    sub_robot_odom_ = nh_.subscribe("/pedsim_simulator/robot_position", 1,
                                    &PedsimSensor::callbackRobotOdom, this);

    transform_listener_ = boost::make_shared<tf::TransformListener>();
  }
  virtual ~PedsimSensor() = default;
  virtual void broadcast() = 0;
  void callbackRobotOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_odom_ = *msg;
    // update sensor anchor.
    fov_->updateViewpoint(robot_odom_.pose.pose.position.x,
                          robot_odom_.pose.pose.position.y);
  }

 protected:
  ros::NodeHandle nh_;
  double rate_ = 25.;
  FoVPtr fov_ = nullptr;
  nav_msgs::Odometry robot_odom_;

  ros::Publisher pub_signals_local_;
  ros::Publisher pub_signals_global_;
  ros::Subscriber sub_robot_odom_;

  boost::shared_ptr<tf::TransformListener> transform_listener_;
};

tf::Pose transformPoint(const tf::StampedTransform& T_r_o,
                        const tf::Vector3& point) {
  tf::Pose source;
  source.setOrigin(point);

  tf::Matrix3x3 identity;
  identity.setIdentity();
  source.setBasis(identity);

  return T_r_o * source;
}

}  // namespace pedsim_ros

#endif

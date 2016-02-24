/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
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
* \author Omar Islas <islas@isir.upmc.fr>
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#include <ros/ros.h>
#include <pedsim_msgs/AgentState.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class JoyTeleop {
public:
    JoyTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void publish();
    ros::NodeHandle ph_, nh_;
    int linear_x, linear_y, deadman_axis_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    pedsim_msgs::AgentState last_published_;
    boost::mutex publish_mutex_;
    bool deadman_pressed_;
    bool zero_twist_published_;
    ros::Timer timer_;
};


JoyTeleop::JoyTeleop()
    : ph_("~")
    , linear_x(2)
    , linear_y(3)
    , deadman_axis_(4)
    , l_scale_(1)
    , a_scale_(0.9)
{
    ph_.param("axis_linear_x", linear_x, linear_x);
    ph_.param("axis_linear_y", linear_y, linear_y);
    ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    deadman_pressed_ = false;
    zero_twist_published_ = false;
    vel_pub_ = ph_.advertise<pedsim_msgs::AgentState>("/pedsim/robot_command", 1, true);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&JoyTeleop::publish, this));
}


void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    pedsim_msgs::AgentState vel;
    deadman_pressed_ = joy->buttons[deadman_axis_];
    vel.type = 2;
    if (deadman_pressed_) {
        // vel.twist.linear.x = l_scale_*joy->axes[linear_x];
        // vel.twist.angular.z = a_scale_*joy->axes[linear_y];
        vel.twist.linear.x = -l_scale_ * joy->axes[linear_x];
        vel.twist.linear.y = l_scale_ * joy->axes[linear_y];
        // double stepx = robot_speed * vx;
        // double stepy = robot_speed * vy;

        last_published_ = vel;
    }
    else if (!deadman_pressed_ && !zero_twist_published_) {
        vel.twist.linear.x = 0;
        vel.twist.angular.z = 0;
        last_published_ = vel;
        zero_twist_published_ = true;
    }
}


void JoyTeleop::publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);
    vel_pub_.publish(last_published_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_teleop_joy");
    JoyTeleop joy_teleop;
    ros::spin();
}

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
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>
#include <termios.h>

#include <pedsim_msgs/AgentState.h>
#include <std_msgs/Header.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

/// -----------------------------------------------------------------
/// \class Teleop
/// \brief Teleoperation interface via Keyboard
/// -----------------------------------------------------------------
class Teleop {
public:
    Teleop();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    double rot_angle_;
    double robot_speed;
    ros::Publisher vel_pub_;
};

Teleop::Teleop()
    : linear_(0)
    , angular_(0)
    , l_scale_(2.0)
    , a_scale_(2.0)
    , rot_angle_(90.0)
    , robot_speed(0.8)
{
    // TODO(?) - add these params to launch file
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<pedsim_msgs::AgentState>("/pedsim/robot_command", 0);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_teleop_keyboard");
    Teleop robot;
    signal(SIGINT, quit);
    robot.keyLoop();
    return (0);
}

// Key loop (for driving the robot)
void Teleop::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ROS_INFO("Robot Teleoperation: Reading from keyboard");
    ROS_INFO("------------------------------------------");
    ROS_INFO("Use arrow keys to move the robot.");
    ROS_INFO("LEFT | RIGHT control direction (15 degree steps)");
    ROS_INFO("UP | DOWN control speed (0.1 m/s steps)");

    // for(;;)
    while (true) {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        // linear_=angular_=0;
        // ROS_DEBUG ( "value: 0x%02X\n", c );

        switch (c) {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            angular_ = 1.0;
            dirty = true;
            rot_angle_ += 15.0;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            angular_ = -1.0;
            dirty = true;
            rot_angle_ -= 15.0;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            dirty = true;
            robot_speed += 0.1;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            dirty = true;
            robot_speed -= 0.1;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("Stop/Pause");
            linear_ = 0.0;
            dirty = true;
            robot_speed = 0.0;
        }

        if (rot_angle_ > 360.0)
            rot_angle_ = acos(cos(rot_angle_));

        if (rot_angle_ < -360.0)
            rot_angle_ = acos(cos(rot_angle_));

        ROS_INFO("Current Speed, Angle [%f, %f]", robot_speed, rot_angle_);

        double vx = cos(rot_angle_ * M_PI / 180.0);
        double vy = sin(rot_angle_ * M_PI / 180.0);
        double stepx = robot_speed * vx;
        double stepy = robot_speed * vy;

        /// TODO - change this to use a spencer-ish message type
        /// to avoid dependence of pedsim_msgs
        pedsim_msgs::AgentState astate;

        std_msgs::Header header_;
        header_.stamp = ros::Time::now();
        astate.header = header_;

        astate.type = 2;
        astate.twist.linear.x = stepx;
        astate.twist.linear.y = stepy;

        if (dirty == true) {
            vel_pub_.publish(astate);
            dirty = false;
        }
    }
    return;
}

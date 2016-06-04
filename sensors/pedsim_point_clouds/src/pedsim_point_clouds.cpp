/*
 * Copyright (c) Social Robotics Laboratory
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Billy Okal <okal@cs.uni-freiburg.de>
 */

#include <tf/transform_listener.h> // must come first due to conflict with Boost signals

/// ros
#include <ros/ros.h>

/// meta
#include <random>
#include <cstdlib>
#include <cmath>

/// data
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

/// -----------------------------------------------------------
/// \class PedsimCloud
/// \brief Receives data from pedsim containing obstacles and
/// persons and published it as point clouds
/// -----------------------------------------------------------
class PedsimCloud {
public:
    PedsimCloud(const ros::NodeHandle& node)
        : nh_(node)
    {
        // set up subscribers
        sub_grid_cells_ = nh_.subscribe("/pedsim/static_obstacles", 1, &PedsimCloud::callbackGridCells, this);
        sub_tracked_persons_ = nh_.subscribe("/pedsim/tracked_persons", 1, &PedsimCloud::callbackTrackedPersons, this);
        sub_robot_odom_ = nh_.subscribe("/pedsim/robot_position", 1, &PedsimCloud::callbackRobotOdom, this);

        // set up publishers
        // publisher for static obstacles as point clouds
        pub_point_cloud_global_ = nh_.advertise<sensor_msgs::PointCloud>("/pedsim/obstacle_cloud_global", 1);
        pub_point_cloud_local_ = nh_.advertise<sensor_msgs::PointCloud>("/pedsim/obstacle_cloud_local", 1);
        // publisher for dynamic obstacles (people) as point clouds
        pub_people_cloud_global_ = nh_.advertise<sensor_msgs::PointCloud>("/pedsim/people_cloud_global", 1);
        pub_people_cloud_local_ = nh_.advertise<sensor_msgs::PointCloud>("/pedsim/people_cloud_local", 1);

        // setup TF listener for obtaining robot position
        transform_listener_ = boost::make_shared<tf::TransformListener>();

        robot_position_.clear();
        robot_position_.resize(2);
        robot_position_ = { 0, 0 };

        robot_frame_ = "odom";

        // read local map dimensions
        nh_.param("/pedsim_point_clouds/local_width", local_width_, 3.0);
        nh_.param("/pedsim_point_clouds/local_height", local_height_, 3.0);
    }
    virtual ~PedsimCloud()
    {
        sub_grid_cells_.shutdown();
        pub_point_cloud_global_.shutdown();
        pub_point_cloud_local_.shutdown();
        pub_people_cloud_global_.shutdown();
        pub_people_cloud_local_.shutdown();
    }

    // control
    void run();

    // subscriber callbacks
    void callbackGridCells(const nav_msgs::GridCells::ConstPtr& msg);
    void callbackTrackedPersons(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);
    void callbackRobotOdom(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh_;

    // robot position
    std::vector<double> robot_position_;

    // local zone around robot (used in local costmaps)
    double local_width_;
    double local_height_;
    std::string robot_frame_;

    // publishers
    ros::Publisher pub_point_cloud_global_;
    ros::Publisher pub_point_cloud_local_;
    ros::Publisher pub_people_cloud_global_;
    ros::Publisher pub_people_cloud_local_;

    // subscribers
    ros::Subscriber sub_grid_cells_;
    ros::Subscriber sub_tracked_persons_;
    ros::Subscriber sub_robot_odom_;

    // Transform listener coverting people poses to be relative to the robot
    boost::shared_ptr<tf::TransformListener> transform_listener_;

protected:
    // check if a point is in the local zone of the robot
    bool inLocalZone(const std::array<double, 2>& point);
};

/// -----------------------------------------------------------
/// \function run
/// \brief Run the node
/// -----------------------------------------------------------
void PedsimCloud::run()
{
    ros::Rate r(30); // Hz
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

/// -----------------------------------------------------------
/// \function inLocalZone
/// \brief Check is a point (e.g. center of person or obstacle)
/// is within the local zone of the robot to be included in the
/// robot's local costmap for planning and other higher level
/// cognition
/// -----------------------------------------------------------
bool PedsimCloud::inLocalZone(const std::array<double, 2>& point)
{
    // NOTE - this hack expands the local map, but that should not cause any
    // issue since we are mainly interested in not missing anythin in the
    // local region
    const double r = std::max(local_width_, local_height_);
    const double dist = std::hypot(robot_position_[0] - point[0], robot_position_[1] - point[1]);

    if (dist <= r)
        return true;
    else
        return false;
}

/// -----------------------------------------------------------
/// \function callbackGridCells
/// \brief Receives grid cells fro pedsim to convert into pcs
/// -----------------------------------------------------------
void PedsimCloud::callbackGridCells(const nav_msgs::GridCells::ConstPtr& msg)
{
    const unsigned int mplex = 200;
    int num_points = msg->cells.size() * mplex;

    // global cloud
    sensor_msgs::PointCloud cloud_global;
    cloud_global.header.stamp = ros::Time::now();
    cloud_global.header.frame_id = "odom";
    cloud_global.points.resize(num_points);
    cloud_global.channels.resize(1);
    cloud_global.channels[0].name = "intensities";
    cloud_global.channels[0].values.resize(num_points);

    // local obstacle cloud
    sensor_msgs::PointCloud cloud_local;
    cloud_local.header.stamp = ros::Time::now();
    cloud_local.header.frame_id = robot_frame_;
    cloud_local.points.resize(num_points);
    cloud_local.channels.resize(1);
    cloud_local.channels[0].name = "intensities";
    cloud_local.channels[0].values.resize(num_points);

    // processing
    std::default_random_engine generator;
    std::uniform_real_distribution<float> float_dist(0, 2);
    std::uniform_real_distribution<float> wide_dist(0, 1);

    // Get the positions of people relative to the robot via TF transform
    tf::StampedTransform tfTransform;
    try {
        transform_listener_->lookupTransform(robot_frame_, msg->header.frame_id, ros::Time(0), tfTransform);
    }
    catch (tf::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from base_footprint to odom failed. Reason: " << e.what());
        return;
    }

    int index = 0;
    for (unsigned int i = 0; i < msg->cells.size(); i++) {
        geometry_msgs::Point cell = msg->cells[i];
        std::array<double, 2> obstacle = { cell.x, cell.y };
        const bool inside = inLocalZone(obstacle);

        for (unsigned int j = 0; j < mplex; j++) {
            // positions relative the robot (local)
            if (inside) {
                tf::Pose source;
                source.setOrigin(tf::Vector3(cell.x + wide_dist(generator), cell.y + wide_dist(generator), 0));
                tf::Matrix3x3 identity;
                identity.setIdentity();
                source.setBasis(identity);
                /// Apply the proper transform
                tf::Pose result = tfTransform * source;

                cloud_local.points[index].x = result.getOrigin().x();
                cloud_local.points[index].y = result.getOrigin().y();
                cloud_local.points[index].z = cell.z + float_dist(generator); // random points in a line
                cloud_local.channels[0].values[index] = 80;
            }

            // global positions
            cloud_global.points[index].x = cell.x + wide_dist(generator);
            cloud_global.points[index].y = cell.y + wide_dist(generator);
            cloud_global.points[index].z = cell.z + float_dist(generator); // random points in a line
            cloud_global.channels[0].values[index] = 50;

            index++;
        }
    }

    // avoid publishing empty local clouds
    if (cloud_local.channels[0].values.size() > 1)
        pub_point_cloud_local_.publish(cloud_local);

    pub_point_cloud_global_.publish(cloud_global);
}

/// -----------------------------------------------------------
/// \function callbackTrackedPersons
/// \brief Receives tracked persons messages and saves them
/// -----------------------------------------------------------
void PedsimCloud::callbackTrackedPersons(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
    const unsigned int mplex = 100;
    int num_points = msg->tracks.size() * mplex;

    // global cloud
    sensor_msgs::PointCloud cloud_global;
    cloud_global.header.stamp = ros::Time::now();
    cloud_global.header.frame_id = "odom";
    cloud_global.points.resize(num_points);
    cloud_global.channels.resize(1);
    cloud_global.channels[0].name = "intensities";
    cloud_global.channels[0].values.resize(num_points);

    // local obstacle cloud
    sensor_msgs::PointCloud cloud_local;
    cloud_local.header.stamp = ros::Time::now();
    cloud_local.header.frame_id = robot_frame_;
    cloud_local.points.resize(num_points);
    cloud_local.channels.resize(1);
    cloud_local.channels[0].name = "intensities";
    cloud_local.channels[0].values.resize(num_points);
    // make some random intensities for the persons
    std::default_random_engine generator;
    std::uniform_int_distribution<int> int_dist(10, 255);
    std::uniform_real_distribution<float> float_dist(0, 1.8);
    std::uniform_real_distribution<float> wide_dist(0, 0.24);

    // Get the positions of people relative to the robot via TF transform
    tf::StampedTransform tfTransform;
    try {
        transform_listener_->lookupTransform(robot_frame_, msg->header.frame_id, ros::Time(0), tfTransform);
    }
    catch (tf::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TFP lookup from base_footprint to odom failed. Reason: " << e.what());
        return;
    }

    int index = 0;
    for (unsigned int i = 0; i < msg->tracks.size(); i++) {
        spencer_tracking_msgs::TrackedPerson p = msg->tracks[i];
        std::array<double, 2> person = { p.pose.pose.position.x, p.pose.pose.position.y };
        const bool inside = inLocalZone(person);

        for (unsigned int j = 0; j < mplex; j++) {
            // positions relative the robot (local)
            if (inside) {
                tf::Pose source;
                source.setOrigin(tf::Vector3(p.pose.pose.position.x + wide_dist(generator),
                    p.pose.pose.position.y + wide_dist(generator),
                    0));
                tf::Matrix3x3 identity;
                identity.setIdentity();
                source.setBasis(identity);
                /// Apply the proper transform
                tf::Pose result = tfTransform * source;

                cloud_local.points[index].x = result.getOrigin().x();
                cloud_local.points[index].y = result.getOrigin().y();
                cloud_local.points[index].z = float_dist(generator); // random points in a line
                cloud_local.channels[0].values[index] = int_dist(generator);
            }

            // global
            cloud_global.points[index].x = p.pose.pose.position.x + wide_dist(generator);
            cloud_global.points[index].y = p.pose.pose.position.y + wide_dist(generator);
            cloud_global.points[index].z = float_dist(generator); // random points in a line
            cloud_global.channels[0].values[index] = int_dist(generator);

            index++;
        }
    }

    // avoid publishing empty local clouds
    if (cloud_local.channels[0].values.size() > 1)
        pub_people_cloud_local_.publish(cloud_local);

    pub_people_cloud_global_.publish(cloud_global);
}

/// -----------------------------------------------------------
/// \function callbackRobotOdom
/// \brief Receives robot position and cache it for use later
/// -----------------------------------------------------------
void PedsimCloud::callbackRobotOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_position_[0] = msg->pose.pose.position.x;
    robot_position_[1] = msg->pose.pose.position.y;

    robot_frame_ = msg->header.frame_id;
}

/// -----------------------------------------------------------
/// main
/// -----------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pedsim_point_clouds");

    ros::NodeHandle n;

    PedsimCloud g(n);
    g.run();

    return EXIT_SUCCESS;
}

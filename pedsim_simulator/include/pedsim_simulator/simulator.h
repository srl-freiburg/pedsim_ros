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

// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AllAgentsState.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/TrackedGroup.h>
#include <pedsim_msgs/TrackedGroups.h>
#include <pedsim_msgs/SocialActivity.h>
#include <pedsim_msgs/SocialActivities.h>

// other ROS-sy messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <animated_marker_msgs/AnimatedMarker.h>
#include <animated_marker_msgs/AnimatedMarkerArray.h>

#include <boost/foreach.hpp>
#include <functional>

#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/scenarioreader.h>
#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/orientationhandler.h>


/// -----------------------------------------------------------------
/// \class Simulator
/// \brief Simulation wrapper
/// \details ROS interface to the scene object provided by pedsim
/// -----------------------------------------------------------------
class Simulator
{
public:
    Simulator(const ros::NodeHandle &node);
    virtual ~Simulator();

    bool initializeSimulation();
    void loadConfigParameters();
    void runSimulation();
	void updateAgentActivities();

    /// publishers
    void publishAgents();
    void publishData();
    void publishSocialActivities();
    void publishGroupVisuals();
    void publishObstacles();
    void publishWalls();
    void publishAttractions();
    void publishRobotPosition();

    /// subscriber helpers
    // Drive robot based on topic messages
    void callbackRobotCommand(const pedsim_msgs::AgentState::ConstPtr &msg);

private:

    // robot agent
    Agent *robot_;

    ros::NodeHandle nh_;

    /// publishers
    // - data messages
    ros::Publisher pub_obstacles_; 			// grid cells
    ros::Publisher pub_all_agents_;			// positions and velocities (old msg)
    ros::Publisher pub_tracked_persons_;	// in spencer format
    ros::Publisher pub_tracked_groups_;
    ros::Publisher pub_social_activities_;

    // - visualization related messages (e.g. markers)
    ros::Publisher pub_attractions_;
    ros::Publisher pub_agent_visuals_;
    ros::Publisher pub_group_lines_;
    ros::Publisher pub_walls_;
    ros::Publisher pub_queues_;
    ros::Publisher pub_waypoints_;
    ros::Publisher pub_agent_arrows_;
    ros::Publisher pub_robot_position_;

    /// subscribers
    ros::Subscriber sub_robot_command_;

    // - Covenient object to handling quaternions
    OrientationHandlerPtr orientation_handler_;

	// agent activity map
	std::map<int, std::string> agent_activities_;

private:

	/// \brief Compute pose of an agent in quaternion format
    inline Eigen::Quaternionf computePose( Agent *a )
	{
        double theta = atan2(a->getvy(), a->getvx());
        Eigen::Quaternionf q = orientation_handler_->rpy2Quaternion(M_PI / 2.0, theta + (M_PI / 2.0), 0.0);
        return q;
    }

	/// \brief Convert agent state machine state to simulated activity
    inline std::string agentStateToActivity( AgentStateMachine::AgentState state )
	{
        std::string activity = "Unknown";

        switch ( state )
		{
        case AgentStateMachine::AgentState::StateWalking:
            activity = pedsim_msgs::AgentState::TYPE_INDIVIDUAL_MOVING;
            break;
        case AgentStateMachine::AgentState::StateGroupWalking:
            activity = pedsim_msgs::AgentState::TYPE_GROUP_MOVING;
            break;
        case AgentStateMachine::AgentState::StateQueueing:
            activity = pedsim_msgs::AgentState::TYPE_WAITING_IN_QUEUE;
            break;
        case AgentStateMachine::AgentState::StateShopping:
            activity = pedsim_msgs::AgentState::TYPE_SHOPPING;
            break;
		case AgentStateMachine::AgentState::StateNone:
			break;
		case AgentStateMachine::AgentState::StateWaiting:
			break;
		}

        // TODO
        // - add standing to the state machine
        // - add waiting at the end of the queue
        // - add group walking

        return activity;
    }

    inline std_msgs::ColorRGBA getColor( int agent_id )
	{
		std::string act = agent_activities_[agent_id];
		std_msgs::ColorRGBA color;

		if ( act == "standing" )
        {
            color.a = 1.0;
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
        }
		else if ( act == "queueing" )
        {
            color.a = 1.0;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 1.0;
        }
		else if ( act == "shopping" )
        {
            color.a = 1.0;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
        }
        else
        {
            color.a = 1.0;
            color.r = 0.0;
            color.g = 0.7;
            color.b = 1.0;
        }

		return color;
	}
};

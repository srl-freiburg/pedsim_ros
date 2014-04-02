// ros and big guys
#include <ros/ros.h>
#include <ros/console.h>

// messages and services
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AllAgentsState.h>

#include <pedsim_srvs/SetAgentState.h>
#include <pedsim_srvs/SetAllAgentsState.h>
#include <pedsim_srvs/GetAgentState.h>
#include <pedsim_srvs/GetAllAgentsState.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#include <pedsim_simulator/orientationhandler.h>

#include <boost/foreach.hpp>

#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/scenarioreader.h>
#include <pedsim_simulator/agentstatemachine.h>


class Simulator
{
public:
	Simulator(const ros::NodeHandle &node);
	virtual ~Simulator();

	bool initializeSimulation();
	void runSimulation();

	/// publishers
	void publishAgentVisuals();
	void publishGroupVisuals();
	void publishObstacles();
	void publishWalls();
	
	/// subscriber helpers
    void callbackRobotCommand ( const pedsim_msgs::AgentState::ConstPtr &msg );

private:

	// robot agent
	Agent* robot_;
	
	ros::NodeHandle nh_;

	// publishers
	ros::Publisher pub_obstacles_;
    ros::Publisher pub_agent_visuals_;
    ros::Publisher pub_group_centers_;
	ros::Publisher pub_group_lines_;
	ros::Publisher pub_walls_;
	ros::Publisher pub_all_agents_;
	
	// subscribers
    ros::Subscriber sub_robot_command_;

    OrientationHandlerPtr orientation_handler_;

};

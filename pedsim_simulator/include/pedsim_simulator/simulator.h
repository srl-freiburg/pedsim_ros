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


class Simulator
{
public:
	Simulator(const ros::NodeHandle &node);
	virtual ~Simulator();

	void runSimulation();

	void publishAgentVisuals();
	void publishGroupVisuals();

private:

	ros::NodeHandle nh_;

	// publishers
    ros::Publisher pub_agent_visuals_;
    ros::Publisher pub_group_centers_;
	ros::Publisher pub_group_lines_;

    OrientationHandlerPtr orientation_handler_;

};

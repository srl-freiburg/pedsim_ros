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
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <pedsim_simulator/scene.h>

#include <pedsim_simulator/simulator.h>

#include <pedsim_simulator/element/agentcluster.h>

#include <QApplication>

Simulator::Simulator ( const ros::NodeHandle &node )
    : nh_ ( node )
{
    // nothing to do here
}

Simulator::~Simulator()
{

}

bool Simulator::initializeSimulation()
{
    /// setup ros publishers
    pub_agent_visuals_ = nh_.advertise<visualization_msgs::MarkerArray> ( "agents_markers", 0 );
    pub_group_centers_ = nh_.advertise<visualization_msgs::MarkerArray> ( "group_centers", 0 );
    pub_agent_arrows_ = nh_.advertise<visualization_msgs::MarkerArray> ( "agent_arrows", 0 );
    pub_group_lines_ = nh_.advertise<visualization_msgs::MarkerArray> ( "group_lines", 0 );
    pub_obstacles_ = nh_.advertise<nav_msgs::GridCells> ( "static_obstacles", 0 );
    pub_walls_ = nh_.advertise<visualization_msgs::Marker> ( "walls", 0 );
    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState> ( "dynamic_obstacles", 0 );
    pub_attractions_ = nh_.advertise<visualization_msgs::Marker> ( "attractions", 0 );
    pub_queues_ = nh_.advertise<visualization_msgs::Marker> ( "queues", 0 );
	pub_waypoints_ = nh_.advertise<visualization_msgs::Marker> ( "waypoints", 0 );

    /// setup any pointers
    orientation_handler_.reset ( new OrientationHandler() );

    /// subscribers
    sub_robot_command_ = nh_.subscribe ( "robot_state", 1, &Simulator::callbackRobotCommand, this );

    /// load parameters
    std::string scene_file_param;
    ros::param::param<std::string> ( "/simulator/scene_file", scene_file_param, "scene.xml" );
    // load scenario file
    // TODO - convert qstrings to std::strings
    QString scenefile = QString::fromStdString ( scene_file_param );

    ScenarioReader scenario_reader;
    bool readResult = scenario_reader.readFromFile ( scenefile );
    if ( readResult == false )
    {
        ROS_WARN ( "Could not load the scene file, check paths" );
        return false;
    }

    ROS_INFO ( "Loading from %s scene file", scene_file_param.c_str() );

    robot_ = nullptr;

	/// load the remaining parameters
    loadConfigParameters();

    return true;
}


/// -----------------------------------------------------------------
/// \brief loadConfigParameters
/// \details Load configuration parameter from ROS parameter server
/// -----------------------------------------------------------------
void Simulator::loadConfigParameters()
{
    double cell_size;
    ros::param::param<double> ( "/simulator/cell_size", cell_size, 1.0 );
    CONFIG.cell_width = cell_size;
    CONFIG.cell_height = cell_size;

    double robot_wait_time;
    ros::param::param<double> ( "/pedsim/move_robot", robot_wait_time, 100.0 );
    CONFIG.robot_wait_time = robot_wait_time;

    double teleop_state;
    ros::param::param<double> ( "/pedsim/teleop_state", teleop_state, 0.0 );
    CONFIG.robot_mode = static_cast<RobotMode> ( teleop_state );
}


/// -----------------------------------------------------------------
/// \brief runSimulation
/// \details Hub of the application
/// -----------------------------------------------------------------
void Simulator::runSimulation()
{
    ros::Rate r ( 25 ); // Hz

    while ( ros::ok() )
    {
		if ( SCENE.getTime() < 0.1 )
		{
			// setup the robot
			BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
			{
				if ( a->getType() == Ped::Tagent::ROBOT )
					robot_ = a;
			}
		}

        SCENE.moveAllAgents();

        publishAgents();

        publishGroupVisuals();

        // obstacle cells (planning needs these)
        publishObstacles();

        // only publish the obstacles in the beginning
        if ( SCENE.getTime() < 10 )
        {
            // attraction visuals
            publishAttractions();

            // visuals for walls
            publishWalls();
        }

        ros::spinOnce();

        r.sleep();
    }
}

/// -----------------------------------------------------------------
/// \brief callbackRobotCommand
/// \details Control the robot based on the set velocity command
/// Listens to incoming messages and manipulates the robot.
/// \param[in] msg Message containing the pos and vel for the robot
/// -----------------------------------------------------------------
void Simulator::callbackRobotCommand ( const pedsim_msgs::AgentState::ConstPtr &msg )
{
    double vx = msg->velocity.x;
    double vy = msg->velocity.y;

    if ( CONFIG.robot_mode == TELEOPERATION || CONFIG.robot_mode == CONTROLLED )
        robot_->setTeleop ( true );

    if ( robot_->getType() == static_cast<Ped::Tagent::AgentType> ( msg->type ) )
    {
		robot_->setvx ( vx );
		robot_->setvy ( vy );
		robot_->setVmax ( sqrt ( vx * vx + vy * vy ) );
    }
}


/// -----------------------------------------------------------------
/// \brief publishAgents
/// \details publish agent status information and the visual markers
/// -----------------------------------------------------------------
void Simulator::publishAgents()
{
    // minor optimization with arrays for speedup
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray arrow_array;

    // status message
    pedsim_msgs::AllAgentsState all_status;
    std_msgs::Header all_header;
    all_header.stamp = ros::Time::now();
    all_status.header = all_header;


    BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
    {
        /// visual marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = a->getId();

        marker.pose.position.x = a->getx();
        marker.pose.position.y = a->gety();
		marker.action = 0;  // add or modify

        // arrow
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time();
        arrow.ns = "pedsim";
        arrow.id = a->getId() + 3000;

        arrow.pose.position.x = a->getx();
        arrow.pose.position.y = a->gety();
        arrow.action = 0;  // add or modify

        arrow.color.a = 1.0;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;

        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;

		if ( robot_ != nullptr &&  a->getType() == robot_->getType() )
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://pedsim_simulator/images/darylbot_rotated_shifted.dae";

            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 1.0;
            marker.color.b = 0.5;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
        }
        else if ( a->getType() == Ped::Tagent::ELDER )
        {
            marker.type = visualization_msgs::Marker::CYLINDER;

            marker.scale.x = 0.4 / 2.0;
            marker.scale.y = 0.4;
            marker.scale.z = 1.75;

            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        // else if ( a->getType() == Ped::Tagent::AIRPORT_CART )
        // {
        //     marker.type = visualization_msgs::Marker::CUBE;

        //     marker.scale.x = 0.4 / 2.0;
        //     marker.scale.y = 0.4;
        //     marker.scale.z = 1.75;

        //     marker.color.a = 1.0;
        //     marker.color.r = 1.0;
        //     marker.color.g = 0.0;
        //     marker.color.b = 1.0;
        // }
        else
		{
			marker.type = visualization_msgs::Marker::CYLINDER;

			marker.scale.x = 0.4 / 2.0;
			marker.scale.y = 0.4;
			marker.scale.z = 1.75;

			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 0.7;
			marker.color.b = 1.0;
		}



        if ( a->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateQueueing )
        {
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        if ( a->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateShopping )
        {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }


        if ( a->getvx() != 0.0 )
        {
            // construct the orientation quaternion
            double theta = atan2 ( a->getvy(), a->getvx() );
            Eigen::Quaternionf q = orientation_handler_->angle2Quaternion ( theta
                                                                          );
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            arrow.pose.orientation.x = q.x();
            arrow.pose.orientation.y = q.y();
            arrow.pose.orientation.z = q.z();
            arrow.pose.orientation.w = q.w();

            double xx = sqrt(a->getvx()*a->getvx() + a->getvy()*a->getvy());
            arrow.scale.x = xx > 0.0 ? xx : 0.01;
        }
        else
        {
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        }

        // set the position in the Z
        marker.pose.position.z = marker.scale.z / 2.0;

        marker_array.markers.push_back ( marker );
        arrow_array.markers.push_back ( arrow );

        /// status message
        pedsim_msgs::AgentState state;
        std_msgs::Header agent_header;
        agent_header.stamp = ros::Time::now();
        state.header = agent_header;

        state.id = a->getId();
        state.type = a->getType();
        state.position.x = a->getx();
        state.position.y = a->gety();
        state.position.z = a->getz();

        state.velocity.x = a->getvx();
        state.velocity.y = a->getvy();
        state.velocity.z = a->getvz();

        AgentStateMachine::AgentState sc =  a->getStateMachine()->getCurrentState();
        state.social_state = static_cast<size_t>( sc );

        all_status.agent_states.push_back ( state );
    }

    // publish the marker array
    pub_agent_visuals_.publish ( marker_array );
    pub_agent_arrows_.publish ( arrow_array );
    pub_all_agents_.publish ( all_status );
}


/// -----------------------------------------------------------------
/// \brief publishGroupVisuals
/// \details publish visualization of groups within the crowd
/// -----------------------------------------------------------------
void Simulator::publishGroupVisuals()
{
    QList<AgentGroup*> groups = SCENE.getGroups();

    visualization_msgs::MarkerArray center_array;

    /// visualize groups (sketchy)
    BOOST_FOREACH ( AgentGroup* ag, groups )
    {
        // skip empty ones
        if ( ag->memberCount() < 1 )
            continue;

        Ped::Tvector gcom = ag->getCenterOfMass();

        // center
        visualization_msgs::Marker center_marker;
        center_marker.header.frame_id = "world";
        center_marker.header.stamp = ros::Time();
        center_marker.ns = "pedsim";
        center_marker.id = ag->getId();

        center_marker.color.a = 0.7;
        center_marker.color.r = 0.0;
        center_marker.color.g = 0.0;
        center_marker.color.b = 1.0;

        center_marker.scale.x = 0.05;
        center_marker.scale.y = 0.05;
        center_marker.scale.z = 0.05;

        center_marker.pose.position.x = gcom.x;
        center_marker.pose.position.y = gcom.y;
        center_marker.pose.position.z = center_marker.scale.z / 2.0;

        center_marker.pose.orientation.x = 0;
        center_marker.pose.orientation.y = 0;
        center_marker.pose.orientation.z = 0;
        center_marker.pose.orientation.w = 1;

        center_marker.type = visualization_msgs::Marker::CYLINDER;

        center_array.markers.push_back ( center_marker );

        // pub_group_centers_.publish ( center_marker );

        // members
        geometry_msgs::Point p1;
        p1.x = gcom.x;
        p1.y = gcom.y;
        p1.z = 0.0;

        visualization_msgs::MarkerArray lines_array;

        BOOST_FOREACH ( Agent* m, ag->getMembers() )
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "pedsim";
            marker.id = m->getId() +1000;

            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.type = visualization_msgs::Marker::ARROW;

            geometry_msgs::Point p2;
            p2.x = m->getx();
            p2.y = m->gety();
            p2.z = 0.0;

            marker.points.push_back ( p1 );
            marker.points.push_back ( p2 );

            lines_array.markers.push_back ( marker );
            // pub_group_lines_.publish ( marker );
        }

        pub_group_centers_.publish ( center_array );
        pub_group_lines_.publish ( lines_array );
    }
}


/// -----------------------------------------------------------------
/// \brief publishObstacles
/// \details publish obstacle cells with information about their
/// positions and cell sizes. Useful for path planning.
/// -----------------------------------------------------------------
void Simulator::publishObstacles()
{
    nav_msgs::GridCells obstacles;
    obstacles.header.frame_id = "world";
    obstacles.cell_width = 1.0;
    obstacles.cell_height = 1.0;

    std::vector<Location>::const_iterator it = SCENE.obstacle_cells_.begin();
    while ( it != SCENE.obstacle_cells_.end() )
    {
        geometry_msgs::Point p;
        Location loc = ( *it );
        p.x = loc.x - 0.5;
        p.y = loc.y - 0.5;
        p.z = 0.0;
        obstacles.cells.push_back ( p );

        it++;
    }

    pub_obstacles_.publish ( obstacles );
}


/// -----------------------------------------------------------------
/// \brief publishWalls
/// \details publish visual markers for obstacle given as 3D cells
/// for visualizing in rviz. Useful for visual plan inspection
/// -----------------------------------------------------------------
void Simulator::publishWalls()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "pedsim";
    marker.id = 10000;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 3.0;

    marker.pose.position.z = marker.scale.z / 2.0;

    marker.type = visualization_msgs::Marker::CUBE_LIST;

    std::vector<Location>::const_iterator it = SCENE.obstacle_cells_.begin();
    while ( it != SCENE.obstacle_cells_.end() )
    {
        geometry_msgs::Point p;
        Location loc = ( *it );
        p.x = loc.x;
        p.y = loc.y;
        p.z = 0.0;
        marker.points.push_back ( p );

        it++;
    }

    pub_walls_.publish ( marker );
}


/// -----------------------------------------------------------------
/// \brief publishAttractions
/// \details publish visual markers for attractions given as 3D cells
/// for visualizing in rviz.
/// -----------------------------------------------------------------
void Simulator::publishAttractions()
{
	/// waypoints
	BOOST_FOREACH ( Waypoint* wp, SCENE.getWaypoints() )
    {
		visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = wp->getId();

        marker.color.a = 0.25;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

		// TODO - get radius information from waypoints
        marker.scale.x = 3.0;
        marker.scale.y = 3.0;
        marker.scale.z = 0.02;

        marker.pose.position.x = wp->getPosition().x;
        marker.pose.position.y = wp->getPosition().y;
        marker.pose.position.z = marker.scale.z / 2.0;

        marker.type = visualization_msgs::Marker::CYLINDER;

		pub_waypoints_.publish ( marker );
    }

    /// publish attractions (shopping areas etc)
    BOOST_FOREACH ( AttractionArea* atr, SCENE.getAttractions() )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = atr->getId();

        marker.color.a = 0.45;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.scale.x = atr->getSize().width();
        marker.scale.y = atr->getSize().height();
        marker.scale.z = 3.0;

        marker.pose.position.x = atr->getPosition().x;
        marker.pose.position.y = atr->getPosition().y;
        marker.pose.position.z = marker.scale.z / 2.0;

        marker.type = visualization_msgs::Marker::CUBE;

        pub_attractions_.publish ( marker );
    }
}



/// -----------------------------------------------------------------
/// \brief main
/// Hub of the application
/// -----------------------------------------------------------------
int main ( int argc, char **argv )
{

    QApplication app ( argc, argv );

    // initialize resources
    ros::init ( argc, argv, "simulator" );


    ros::NodeHandle node;


    Simulator sm ( node );

    if ( sm.initializeSimulation() )
    {
		ROS_INFO ( "node initialized, now running " );

        sm.runSimulation();
    }
    else
    {
        return EXIT_FAILURE;
    }


//     double x1, x2, y1, y2;
//     ros::param::param<double> ( "/pedsim/x1", x1, 0.0 );
//     ros::param::param<double> ( "/pedsim/x2", x2, 100.0 );
//     ros::param::param<double> ( "/pedsim/y1", y1, 0.0 );
//     ros::param::param<double> ( "/pedsim/y2", y2, 100.0 );
//
//     // Scene sim_scene(0, 0, 45, 45, node);
//     // Scene sim_scene(0, 0, 300, 100, node);
//     Scene sim_scene ( x1, y1, x2, y2, node );


    return app.exec();
}

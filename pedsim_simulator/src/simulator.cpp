

#include <pedsim_simulator/scene.h>

#include <pedsim_simulator/simulator.h>

#include <pedsim_simulator/element/agentcluster.h>

#include <QApplication>

Simulator::Simulator(const ros::NodeHandle &node)
	: nh_(node)
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
    pub_group_centers_ = nh_.advertise<visualization_msgs::Marker> ( "group_centers", 0 );
    pub_group_lines_ = nh_.advertise<visualization_msgs::Marker> ( "group_lines", 0 );
    pub_obstacles_ = nh_.advertise<nav_msgs::GridCells> ( "static_obstacles", 0 );
    pub_walls_ = nh_.advertise<visualization_msgs::Marker> ( "walls", 0 );
	pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState> ( "dynamic_obstacles", 0 );
	pub_attractions_ = nh_.advertise<visualization_msgs::Marker> ( "attractions", 0 );
	pub_queues_ = nh_.advertise<visualization_msgs::Marker> ( "queues", 0 );
	
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

	ScenarioReader scenarioReader;
	bool readResult = scenarioReader.readFromFile(scenefile);
	if(readResult == false) {
		ROS_WARN ( "Could not load the scene file, check paths" );
		return false;
	}

    ROS_INFO ( "Loading from %s scene file", scene_file_param.c_str() );
	
	robot_ = nullptr;
	
	return true;
}


void Simulator::runSimulation()
{
	// setup the robot 
    BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
    {
		// TODO - convert back to robot type enum
        if ( a->getType() == 2 )
            robot_ = a;
    }

    ros::Rate r ( 30 ); // Hz

    while ( ros::ok() )
    {
		SCENE.moveAllAgents();

		publishAgentVisuals();

        publishGroupVisuals();

        // visuals for walls
        publishWalls();

        // only publish the obstacles in the beginning
        if ( SCENE.getTime() < 100 )
            publishObstacles();

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

//     if ( CONFIG.robot_mode == TELEOPERATION )
//         robot_->setteleop ( true );

    if ( robot_->getType() == msg->type )
    {
        if ( robot_->getTeleop() == false )
        {
            robot_->setvx ( vx );
            robot_->setvy ( vy );
            robot_->setVmax ( sqrt ( vx * vx + vy * vy ) );
        }
        else
        {
            robot_->setvx ( vx );
            robot_->setvy ( vy );
            robot_->setVmax ( sqrt ( vx * vx + vy * vy ) );
        }
    }
}


/// -----------------------------------------------------------------
/// \brief publishAgentVisuals
/// \details publish agent status information and the visual markers
/// -----------------------------------------------------------------
void Simulator::publishAgentVisuals()
{
    // minor optimization with arrays for speedup
    visualization_msgs::MarkerArray marker_array;

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

        marker.type = visualization_msgs::Marker::CYLINDER;
// 		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//         marker.mesh_resource = "package://pedsim_simulator/images/zoey/girl.dae";
		
		marker.action = 0;  // add or modify

		marker.scale.x = 0.3 / 2.0;
        marker.scale.y = 0.3;
        marker.scale.z = 1.75;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.7;
        marker.color.b = 1.0;

        marker.pose.position.z = marker.scale.z / 2.0;
        marker.pose.position.x = a->getx();
        marker.pose.position.y = a->gety();

        if ( a->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateQueueing)
        {			
            marker.color.a = 1.0;
            marker.color.r = 1.0;
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
        }
        else
        {
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        }
        marker_array.markers.push_back ( marker );
		
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

        all_status.agent_states.push_back ( state );
    }

    // publish the marker array
    pub_agent_visuals_.publish ( marker_array );
	pub_all_agents_.publish ( all_status );
}


/// -----------------------------------------------------------------
/// \brief publishGroupVisuals
/// \details publish visualization of groups within the crowd
/// -----------------------------------------------------------------
void Simulator::publishGroupVisuals()
{
    QList<AgentGroup*> groups = SCENE.getGroups();

    /// visualize groups (sketchy)
    BOOST_FOREACH ( AgentGroup* ag, groups )
    {
        // skip empty ones
        if ( ag->memberCount() < 1)
         continue;

        // ag->updateCenterOfMass();
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

        center_marker.scale.x = 0.2;
        center_marker.scale.y = 0.2;
        center_marker.scale.z = 0.2;

        center_marker.pose.position.x = gcom.x;
        center_marker.pose.position.y = gcom.y;
        center_marker.pose.position.z = center_marker.scale.z / 2.0;

        center_marker.pose.orientation.x = 0;
        center_marker.pose.orientation.y = 0;
        center_marker.pose.orientation.z = 0;
        center_marker.pose.orientation.w = 1;

        center_marker.type = visualization_msgs::Marker::CYLINDER;

        pub_group_centers_.publish ( center_marker );

        // members
        geometry_msgs::Point p1;
        p1.x = gcom.x;
        p1.y = gcom.y;
        p1.z = 0.0;

        BOOST_FOREACH ( Agent* m, ag->getMembers() )
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "pedsim";
            marker.id = m->getId()+1000;

            marker.color.a = 0.7;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.type = visualization_msgs::Marker::ARROW;

            geometry_msgs::Point p2;
            p2.x = m->getx();
            p2.y = m->gety();
            p2.z = 0.0;

            marker.points.push_back ( p1 );
            marker.points.push_back ( p2 );

            pub_group_lines_.publish ( marker );
        }
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
        // p.x = loc.x + (cell_size/2.0f);
        // p.y = loc.y + (cell_size/2.0f);
        p.x = loc.x;
        p.y = loc.y;
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
	QMap<QString, AttractionArea*>::iterator it = SCENE.getAttractions().begin();
	
	while ( it !=  SCENE.getAttractions().end() )
	{
		AttractionArea* atr = it.value();
		
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time();
		marker.ns = "pedsim";
		marker.id = 2000;

		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 3.0;

		marker.pose.position.x = atr->getPosition().x;
		marker.pose.position.y = atr->getPosition().y;
		marker.pose.position.z = marker.scale.z / 2.0;

		marker.type = visualization_msgs::Marker::CUBE;
		
		pub_attractions_.publish( marker );
		
		it++;
	}
}



/// -----------------------------------------------------------------
/// \brief main
/// Hub of the application
/// -----------------------------------------------------------------
int main ( int argc, char **argv )
{

	QApplication app(argc, argv);

    // initialize resources
    ros::init ( argc, argv, "simulator" );

    ROS_INFO ( "node initialized" );

    ros::NodeHandle node;


	Simulator sm(node);
	
	if ( sm.initializeSimulation() )
	{
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

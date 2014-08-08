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
    delete robot_;

    // shutdown service servers and publishers
    sub_robot_command_.shutdown();
    pub_agent_visuals_.shutdown();
    pub_agent_arrows_.shutdown();
    pub_group_lines_.shutdown();
    pub_walls_.shutdown();
    pub_attractions_.shutdown();
    pub_queues_.shutdown();
    pub_waypoints_.shutdown();
    pub_obstacles_.shutdown();
    pub_all_agents_.shutdown();
    pub_tracked_persons_.shutdown();
    pub_tracked_groups_.shutdown();
    pub_social_activities_.shutdown();

    int returnValue = 0;
    QCoreApplication::exit(returnValue);
}

bool Simulator::initializeSimulation()
{
    /// setup ros publishers
    // visualizations
    pub_agent_visuals_ = nh_.advertise<animated_marker_msgs::AnimatedMarkerArray> ( "agents_markers", 0 );
    pub_agent_arrows_ = nh_.advertise<visualization_msgs::MarkerArray> ( "agent_arrows", 0 );
    pub_group_lines_ = nh_.advertise<visualization_msgs::MarkerArray> ( "group_lines", 0 );
    pub_walls_ = nh_.advertise<visualization_msgs::Marker> ( "walls", 0 );
    pub_attractions_ = nh_.advertise<visualization_msgs::Marker> ( "attractions", 0 );
    pub_queues_ = nh_.advertise<visualization_msgs::Marker> ( "queues", 0 );
    pub_waypoints_ = nh_.advertise<visualization_msgs::Marker> ( "waypoints", 0 );

    // informative topics (data)
    pub_obstacles_ = nh_.advertise<nav_msgs::GridCells> ( "static_obstacles", 0 );
    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState> ( "dynamic_obstacles", 0 );

    pub_tracked_persons_ = nh_.advertise<spencer_tracking_msgs::TrackedPersons> ( "/spencer/perception/tracked_persons", 0 );
    pub_tracked_groups_ = nh_.advertise<spencer_tracking_msgs::TrackedGroups> ( "/spencer/perception/tracked_groups", 0 );
    pub_social_activities_ = nh_.advertise<spencer_social_relation_msgs::SocialActivities> ( "/spencer/perception/social_activities", 0 );

    /// setup any pointers
    orientation_handler_.reset ( new OrientationHandler() );
    robot_ = nullptr;

    /// subscribers
    sub_robot_command_ = nh_.subscribe ( "/pedsim_simulator/robot_command", 1, &Simulator::callbackRobotCommand, this );

    /// load parameters
    std::string scene_file_param;
    ros::param::param<std::string> ( "/simulator/scene_file", scene_file_param, "scene.xml" );
    // load scenario file
    QString scenefile = QString::fromStdString ( scene_file_param );
    ScenarioReader scenario_reader;
    bool readResult = scenario_reader.readFromFile ( scenefile );
    if ( readResult == false )
    {
        ROS_WARN ( "Could not load the scene file, check paths" );
        return false;
    }

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
	double robot_wait_time;
    ros::param::param<double> ( "/pedsim_simulator/robot_wait_time", robot_wait_time, 10.0 );
	CONFIG.robot_wait_time = robot_wait_time;
	
	double max_robot_speed;
	ros::param::param<double> ( "/pedsim_simulator/max_robot_speed", max_robot_speed, 2.0 );
	CONFIG.max_robot_speed = max_robot_speed;
	
    double teleop_flag;
    ros::param::param<double> ( "/pedsim_simulator/teleop_flag", teleop_flag, 0.0 );
    CONFIG.robot_mode = static_cast<RobotMode> ( teleop_flag );
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

        publishAgents();    // TODO - remove this old piece
        publishData();
        publishSocialActivities();
        publishGroupVisuals();

        // obstacle cells (planning needs these)
        publishObstacles();

        // only publish the obstacles in the beginning
        if ( SCENE.getTime() < 10 )
        {
            publishAttractions();
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

    if ( CONFIG.robot_mode == RobotMode::TELEOPERATION || CONFIG.robot_mode == RobotMode::CONTROLLED )
        robot_->setTeleop ( true );

    if ( robot_->getType() == static_cast<Ped::Tagent::AgentType> ( msg->type ) )
    {
		robot_->setvx ( vx );
		robot_->setvy ( vy );
        // NOTE - check if this is really necessary
// 		robot_->setVmax ( sqrt ( vx * vx + vy * vy ) );
		robot_->setVmax( CONFIG.max_robot_speed );
    }
}


/// -----------------------------------------------------------------
/// \brief publishSocialActivities
/// \details publish spencer_relation_msgs::SocialActivities
/// -----------------------------------------------------------------
void Simulator::publishSocialActivities()
{
    /// Social activities
    spencer_social_relation_msgs::SocialActivities social_activities;
    std_msgs::Header social_activities_header;
    social_activities_header.stamp = ros::Time::now();
    social_activities.header = social_activities_header;

    spencer_social_relation_msgs::SocialActivity queueing_activity;
    spencer_social_relation_msgs::SocialActivity shopping_activity;
    spencer_social_relation_msgs::SocialActivity standing_activity;
    spencer_social_relation_msgs::SocialActivity group_moving_activity;
    spencer_social_relation_msgs::SocialActivity individual_moving_activity;

    BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
    {
        /// activity of the current agent
        AgentStateMachine::AgentState sact =  a->getStateMachine()->getCurrentState();

        if ( sact == AgentStateMachine::AgentState::StateQueueing )
        {
            queueing_activity.type = spencer_social_relation_msgs::SocialActivity::TYPE_WAITING_IN_QUEUE;
            queueing_activity.confidence = 1.0;
            queueing_activity.track_ids.push_back( a->getId() );
        }

        if ( sact == AgentStateMachine::AgentState::StateShopping )
        {
            shopping_activity.type = spencer_social_relation_msgs::SocialActivity::TYPE_SHOPPING;
            shopping_activity.confidence = 1.0;
            shopping_activity.track_ids.push_back( a->getId() );
        }

        if ( a->getType() == Ped::Tagent::ELDER )  // Hack for really slow people
        {
            standing_activity.type = spencer_social_relation_msgs::SocialActivity::TYPE_STANDING;
            standing_activity.confidence = 1.0;
            standing_activity.track_ids.push_back( a->getId() );
        }

        if ( sact == AgentStateMachine::AgentState::StateGroupWalking )
        {
            group_moving_activity.type = spencer_social_relation_msgs::SocialActivity::TYPE_GROUP_MOVING;
            group_moving_activity.confidence = 1.0;
            group_moving_activity.track_ids.push_back( a->getId() );
        }

        if ( sact == AgentStateMachine::AgentState::StateWalking )
        {
            individual_moving_activity.type = spencer_social_relation_msgs::SocialActivity::TYPE_INDIVIDUAL_MOVING;
            individual_moving_activity.confidence = 1.0;
            individual_moving_activity.track_ids.push_back( a->getId() );
        }
        // else
        //     continue;
    }

    social_activities.elements.push_back( queueing_activity );
    social_activities.elements.push_back( shopping_activity );
    social_activities.elements.push_back( standing_activity );
    social_activities.elements.push_back( group_moving_activity );
    social_activities.elements.push_back( individual_moving_activity );

    pub_social_activities_.publish( social_activities );
}


/// -----------------------------------------------------------------
/// \brief publishData
/// \details publish tracked persons and tracked groups messages
/// -----------------------------------------------------------------
void Simulator::publishData()
{
    /// Tracked people
    spencer_tracking_msgs::TrackedPersons tracked_people;
    std_msgs::Header tracked_people_header;
    tracked_people_header.stamp = ros::Time::now();
    tracked_people.header = tracked_people_header;

    BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
    {
        spencer_tracking_msgs::TrackedPerson person;
        person.track_id = a->getId();
        person.is_occluded = false;
        // person.detection_id = 0;  // not simulated yet
        // person.age = 0;   // also not simulated yet

        double theta = atan2 ( a->getvy(), a->getvx() );
        Eigen::Quaternionf q = orientation_handler_->angle2Quaternion ( theta );

        geometry_msgs::PoseWithCovariance pcov;
        pcov.pose.position.x = a->getx();
        pcov.pose.position.y = a->gety();
        pcov.pose.position.z = 0.0;
        pcov.pose.orientation.x = q.x();
        pcov.pose.orientation.y = q.y();
        pcov.pose.orientation.z = q.z();
        pcov.pose.orientation.w = q.w();
        person.pose = pcov;

        // TODO - recheck this
        geometry_msgs::TwistWithCovariance tcov;
        tcov.twist.linear.x = a->getvx();
        tcov.twist.linear.y = a->getvy();
        tcov.twist.linear.z = 0.0;
        // tcov.twist.angular.x = 0;
        // tcov.twist.angular.y = 0;
        // tcov.twist.angular.z = 0;
        person.twist = tcov;

        tracked_people.tracks.push_back( person );
    }

    /// Tracked groups
    spencer_tracking_msgs::TrackedGroups tracked_groups;
    std_msgs::Header tracked_groups_header;
    tracked_groups_header.stamp = ros::Time::now();
    tracked_groups.header = tracked_groups_header;

    QList<AgentGroup*> sim_groups = SCENE.getGroups();
    BOOST_FOREACH ( AgentGroup* ag, sim_groups )
    {
        spencer_tracking_msgs::TrackedGroup group;
        group.group_id = ag->getId();
        // group.age = 0; //NOTE  not simulated so far
        Ped::Tvector com = ag->getCenterOfMass();
        // group.centerOfGravity = ... // TODO - convert CoM to Pose with Covariance

        BOOST_FOREACH ( Agent* m, ag->getMembers() )
        {
            group.track_ids.push_back(m->getId());
        }

        tracked_groups.groups.push_back(group);
    }

    /// publish the messages
    pub_tracked_persons_.publish( tracked_people );
    pub_tracked_groups_.publish( tracked_groups );
}


/// -----------------------------------------------------------------
/// \brief publishAgents
/// \details publish agent status information and the visual markers
/// \note This method is old format and is deprecated
/// -----------------------------------------------------------------
void Simulator::publishAgents()
{
    animated_marker_msgs::AnimatedMarkerArray marker_array;
    visualization_msgs::MarkerArray arrow_array;

    // status message
    pedsim_msgs::AllAgentsState all_status;
    std_msgs::Header all_header;
    all_header.stamp = ros::Time::now();
    all_status.header = all_header;

    BOOST_FOREACH ( Agent* a, SCENE.getAgents() )
    {
        /// walking people message
        animated_marker_msgs::AnimatedMarker marker;
        marker.mesh_use_embedded_materials = true;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = a->getId();
        marker.type = animated_marker_msgs::AnimatedMarker::MESH_RESOURCE;
        marker.mesh_resource = "package://pedsim_simulator/images/animated_walking_man.mesh";

        marker.pose.position.x = a->getx();
        marker.pose.position.y = a->gety();
		marker.action = 0;  // add or modify
        const double person_scale = 2.0 / 8.5 * 1.8;  // TODO - move these magic numbers to a config file
        marker.scale.x = person_scale; marker.scale.y = person_scale; marker.scale.z = person_scale;

        /// arrows
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time();
        arrow.ns = "pedsim";
        arrow.id = a->getId() + 3000;

        arrow.pose.position.x = a->getx();
        arrow.pose.position.y = a->gety();
        arrow.pose.position.z = 1.4;
        arrow.action = 0;  // add or modify
        arrow.color.a = 1.0; arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0;
        arrow.scale.y = 0.05; arrow.scale.z = 0.05;

        if ( a->getType() == Ped::Tagent::ELDER )
        {
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
        }
        else
		{
			marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.7; marker.color.b = 1.0;
		}

        if ( a->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateQueueing )
        {
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0;
        }

        if ( a->getStateMachine()->getCurrentState() == AgentStateMachine::AgentState::StateShopping )
        {
            marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
        }


        Eigen::Quaternionf q = computePose( a );
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        double theta = atan2 ( a->getvy(), a->getvx() );
        Eigen::Quaternionf qa = orientation_handler_->angle2Quaternion ( theta );

        if ( a->getvx() != 0.0 )
        {
            arrow.pose.orientation.x = qa.x();
            arrow.pose.orientation.y = qa.y();
            arrow.pose.orientation.z = qa.z();
            arrow.pose.orientation.w = qa.w();

            double xx = sqrt(a->getvx()*a->getvx() + a->getvy()*a->getvy());
            arrow.scale.x = xx > 0.0 ? xx : 0.01;

            marker.animation_speed = xx * 0.7;
        }
        else
        {
            marker.animation_speed = 0.0;
        }

        if ( robot_ != nullptr &&  a->getType() == robot_->getType() )
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://pedsim_simulator/images/darylbot_rotated_shifted.dae";
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
            marker.scale.x = 0.8; marker.scale.y = 0.8; marker.scale.z = 1.0;

            marker.pose.orientation.x = qa.x();
            marker.pose.orientation.y = qa.y();
            marker.pose.orientation.z = qa.z();
            marker.pose.orientation.w = qa.w();

            marker.pose.position.z = 0.7;
            arrow.pose.position.z = 1.0;
        }

        marker_array.markers.push_back ( marker );
        arrow_array.markers.push_back ( arrow );

        /// status message
        /// TODO - remove this once, we publish internal states using
        /// spencer messages
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
        // state.social_state = static_cast<size_t>( sc );
        state.social_state = agentStateToActivity ( sc );

        if ( a->getType() == Ped::Tagent::ELDER )
            state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;

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

    /// visualize groups (sketchy)
    BOOST_FOREACH ( AgentGroup* ag, groups )
    {
        // skip empty ones
        if ( ag->memberCount() < 1 )
            continue;

        /// members of the group
        geometry_msgs::Point p1;
        Ped::Tvector gcom = ag->getCenterOfMass();
        p1.x = gcom.x; p1.y = gcom.y; p1.z = 1.4;
        visualization_msgs::MarkerArray lines_array;

        BOOST_FOREACH ( Agent* m, ag->getMembers() )
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "pedsim";
            marker.id = m->getId() +1000;

            marker.color.a = 0.7; marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
            marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05;
            marker.type = visualization_msgs::Marker::ARROW;
            geometry_msgs::Point p2;
            p2.x = m->getx(); p2.y = m->gety(); p2.z = 1.4;

            marker.points.push_back ( p1 );
            marker.points.push_back ( p2 );
            lines_array.markers.push_back ( marker );
        }

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
        p.x = loc.x - 0.5; p.y = loc.y - 0.5; p.z = 0.0;
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
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 3.0;
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
// 	BOOST_FOREACH ( Waypoint* wp, SCENE.getWaypoints() )
//     {
// // 		wp->getType()
// 		visualization_msgs::Marker marker;
//         marker.header.frame_id = "world";
//         marker.header.stamp = ros::Time();
//         marker.ns = "pedsim";
//         marker.id = wp->getId();
//
//         marker.color.a = 0.15;
//         marker.color.r = 1.0;
//         marker.color.g = 0.0;
//         marker.color.b = 1.0;
//
// 		// TODO - get radius information from waypoints
//         marker.scale.x = 3.0;
//         marker.scale.y = 3.0;
//         marker.scale.z = 0.02;
//
//         marker.pose.position.x = wp->getPosition().x;
//         marker.pose.position.y = wp->getPosition().y;
//         marker.pose.position.z = marker.scale.z / 2.0;
//
//         marker.type = visualization_msgs::Marker::CYLINDER;
//
// 		pub_waypoints_.publish ( marker );
//     }

 //    WaitingQueue* info_desk  = SCENE.getWaitingQueueByName("klm");
	// visualization_msgs::Marker marker;
	// marker.header.frame_id = "world";
	// marker.header.stamp = ros::Time();
	// marker.ns = "pedsim";
	// marker.id = info_desk->getId();
	// marker.type = visualization_msgs::Marker::MESH_RESOURCE;
 //    marker.mesh_resource = "package://pedsim_simulator/images/kiosk.dae";

	// marker.scale.x = 1.0;
	// marker.scale.y = 1.0;
	// marker.scale.z = 1.0;

	// marker.pose.position.x = info_desk->getPosition().x - 2.5;
	// marker.pose.position.y = info_desk->getPosition().y + 0.5;
	// marker.pose.position.z = 0.0; //marker.scale.z / 2.0;

	// pub_queues_.publish ( marker );


	// WaitingQueue* kq  = SCENE.getWaitingQueueByName("kq");
	// visualization_msgs::Marker marker2;
	// marker2.header.frame_id = "world";
	// marker2.header.stamp = ros::Time();
	// marker2.ns = "pedsim";
	// marker2.id = kq->getId();
	// marker2.type = visualization_msgs::Marker::MESH_RESOURCE;
 //    marker2.mesh_resource = "package://pedsim_simulator/images/kiosk.dae";

	// marker2.scale.x = 1.0;
	// marker2.scale.y = 1.0;
	// marker2.scale.z = 1.0;

	// marker2.pose.position.x = kq->getPosition().x - 2.5;
	// marker2.pose.position.y = kq->getPosition().y + 0.5;
	// marker2.pose.position.z = 0.0; //marker.scale.z / 2.0;

	// pub_queues_.publish ( marker2 );


    /// publish attractions (shopping areas etc)
    BOOST_FOREACH ( AttractionArea* atr, SCENE.getAttractions() )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = atr->getId();

        marker.color.a = 0.35;
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
        ROS_WARN("Could not initialize simulation, aborting");
        return EXIT_FAILURE;
    }

    return app.exec();
}

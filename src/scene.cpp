
#include "scene.h"

Scene::Scene(const ros::NodeHandle& node)  
    : Ped::Tscene(), nh_(node)
{
    // useful for keeping track of agents in the cleaning process
    tree = new Ped::Ttree(this, 0, -20, -20, 1000, 1000);
}

Scene::Scene( double left, double up, double width, double height, const ros::NodeHandle& node )
    : Ped::Tscene(left, up, width, height), nh_(node)
{
    // useful for keeping track of agents in the cleaning process
    tree = new Ped::Ttree(this, 0, -20, -20, 1000, 1000);
}


bool Scene::srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request& req, pedsim_srvs::SetAgentState::Response& res)
{
    pedsim_msgs::AgentState state = req.state;

    double vx = state.velocity.x;
    double vy = state.velocity.y;

    if (robot_->getid() == state.id)  
    {
        robot_->setvx( vx );
        robot_->setvy( vy );
        robot_->setVmax( sqrt( vx * vx + vy * vy ) );
    }
    else
    {
        res.finished = false;
    }

    res.finished = true;

    return res.finished;
}



void Scene::cleanupItems() 
{
    cleanup();
}

void Scene::clear() 
{
    all_agents_.clear();

    BOOST_FOREACH(Waypoint* waypoint, waypoints)
        delete waypoint;
    waypoints.clear();

    BOOST_FOREACH(Obstacle* obs, obstacles)
        delete obs;
    obstacles.clear();
}

void Scene::runSimulation() 
{

    /// setup the agents and the robot
    all_agents_ = getAllAgents();

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);

        if (a->gettype() == 2)
            robot_ = a;

        // setup for teleoperation
        double teleop_state;
        nh_.getParam("/irl_features/teleop_state", teleop_state);
        if (teleop_state == 0.0)
            robot_->setteleop(false);
        else
            robot_->setteleop(true);
    }

    ros::Rate r( 1 /  CONFIG.simh );
    // ros::Rate r( 15 ); // 15 Hz seems to be good visually

    while (ros::ok())
    {
        moveAllAgents();
        // spawnKillAgents();

        publishAgentStatus();
        publishAgentVisuals();

        // only publish the obstacles in the beginning
        if (timestep_ < 200)
            publishObstacles();

        // helps to make things faster
        cleanupItems();

        ros::spinOnce();

        r.sleep();
    }
}

bool Scene::initialize()
{
    // start the time steps
    timestep_ = 0;

    /// setup the list of all agents and the robot agent
    all_agents_.clear();
    obstacle_cells_.clear();

    // setup publishers
    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState>("AllAgentsStatus", 0);
    pub_agent_visuals_ = nh_.advertise<visualization_msgs::MarkerArray>( "agents_markers", 0 );
    pub_obstacles_ = nh_.advertise<nav_msgs::GridCells>( "static_obstacles", 0 );

    // subscribers
    sub_robot_state_ = nh_.subscribe("robot_state", 1, &Scene::callbackRobotState, this);

    // services hooks
    srv_move_agent_ = nh_.advertiseService("SetAgentState", &Scene::srvMoveAgentHandler, this);


    // load parameters
    std::string scene_file_param;
    nh_.getParam("/simulator/scene_file", scene_file_param);
    double cell_size;
    nh_.getParam("/irl_features/cell_size", cell_size);
    CONFIG.width = cell_size;
    CONFIG.height = cell_size;

    ROS_INFO("Loading from %s scene file", scene_file_param.c_str());

    // load scenario file
    QString scenefile = QString::fromStdString(scene_file_param);
    if (!readFromFile(scenefile)) 
    {
        ROS_WARN("Could not load the scene file, check paths");
        return false;
    }

    return true;
}


void Scene::moveAllAgents()
{
    timestep_++;

    // move the agents by social force
    Ped::Tscene::moveAgents(CONFIG.simh);
}



void Scene::callbackRobotState(const pedsim_msgs::AgentState::ConstPtr& msg)
{
    double vx = msg->velocity.x;
    double vy = msg->velocity.y;

    double robot_state;
    nh_.getParam("/irl_features/move_robot", robot_state);
    
    double teleop_state;
    nh_.getParam("/irl_features/teleop_state", teleop_state);
    
    if (timestep_ >= robot_state)
    {
        if (robot_->getid() == msg->id)  
        {

            if (teleop_state == 0.0)
            {
                robot_->setvx( vx );
                robot_->setvy( vy );
                robot_->setVmax( 1.34 );
                // robot_->setVmax( sqrt( vx * vx + vy * vy ) );
            } 
            else 
            {
                robot_->setvx( vx );
                robot_->setvy( vy );
                // robot_->setVmax( sqrt( vx * vx + vy * vy ) );
                robot_->setVmax( 1.5 );
            }

            // ROS_DEBUG("(rx, ry), (%f, %f)", robot_->getx(), robot_->gety());
        }

    } else
    {
        if (robot_->getid() == msg->id)  
        {
            // robot_->setvx( vx );
            // robot_->setvy( vy );
            robot_->setVmax( 0.0 );
        }
    }
}


void Scene::publishAgentStatus()
{
    pedsim_msgs::AllAgentsState all_status;
    std_msgs::Header all_header;
    all_header.stamp = ros::Time::now();
    all_status.header = all_header;

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) 
    {
        Ped::Tagent *a = (*iter);

        pedsim_msgs::AgentState state;
        std_msgs::Header agent_header;
        agent_header.stamp = ros::Time::now();
        state.header = agent_header;

        state.id = a->getid();
        state.type = a->gettype();
        state.position.x = a->getx();
        state.position.y = a->gety();
        state.position.z = a->getz();

        state.velocity.x = a->getvx();
        state.velocity.y = a->getvy();
        state.velocity.z = a->getvz();

        all_status.agent_states.push_back(state);
    }

    pub_all_agents_.publish(all_status);
}


void Scene::publishAgentVisuals()
{

    // minor optimization with arrays for speedup
    visualization_msgs::MarkerArray marker_array;

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = a->getid();

        if (a->gettype() == robot_->gettype()) 
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://simulator/images/darylbot.dae";
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            // marker.type = visualization_msgs::Marker::CYLINDER;
            // marker.color.a = 1.0;
            // marker.color.r = 0.0;
            // marker.color.g = 0.0;
            // marker.color.b = 1.0;

            // marker.scale.x = 0.3;
            // marker.scale.y = 0.3;
            // marker.scale.z = 1.5;
        }
        else
        {
            marker.type = visualization_msgs::Marker::CYLINDER;
            
            if (a->gettype() == 0)
            {
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
            }
            else
            {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            

            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 1.5;
        }

        marker.action = 0;  // add or modify
        marker.pose.position.x = a->getx();
        marker.pose.position.y = a->gety();
        marker.pose.position.z = 0.75;

        if (a->getvx() != 0.0) 
        {
            // construct the orientation quaternion
            double theta = atan2(a->getvy(), a->getvx());
            double* q = angleToQuaternion(theta);

            marker.pose.orientation.x = q[0];
            marker.pose.orientation.y = q[1];
            marker.pose.orientation.z = q[2];
            marker.pose.orientation.w = q[3];
        }
        else
        {
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        }
        // pub_agent_visuals_.publish( marker );
        marker_array.markers.push_back( marker );

    }


    // now publish the array
    pub_agent_visuals_.publish( marker_array );
}


void Scene::publishObstacles()
{
    nav_msgs::GridCells obstacles;
    obstacles.header.frame_id = "world";
    obstacles.cell_width = CONFIG.width;
    obstacles.cell_height = CONFIG.height;

    double cell_size;
    nh_.getParam("/simulator/cell_size", cell_size);

    std::vector<TLoc>::const_iterator it = obstacle_cells_.begin();
    while( it != obstacle_cells_.end())
    {
        geometry_msgs::Point p;
        TLoc loc = (*it);
        // p.x = loc.x + (cell_size/2.0f);
        // p.y = loc.y + (cell_size/2.0f);
        p.x = loc.x;
        p.y = loc.y;
        p.z = 0.0;
        obstacles.cells.push_back(p);

        it++;
    }

    pub_obstacles_.publish(obstacles);
}


void Scene::spawnKillAgents() 
{
    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) 
    {
        Ped::Tagent *a = (*iter);
        double ax = a->getx();
        double ay = a->gety();

  
        if (a->gettype() != 2 && timestep_ > 10) 
        {
            if (a->destination->gettype() == Ped::Twaypoint::TYPE_DEATH)
            {
                // get the agents first ever waypoint and move it there to start over
                Ped::Twaypoint* wp = a->birth_waypoint;
                Ped::Twaypoint* wc = a->death_waypoint;

                double wx = wc->getx();
                double wy = wc->gety();

                if (sqrt( pow(ax-wx, 2.0) + pow(ay-wy, 2.0) )  <= ((wc->getr())) ) 
                {
                // if (a->hasreacheddestination) {
                    double randomizedX = wp->getx();
                    double randomizedY = wp->gety();
                    double dx = wp->getr();
                    double dy = wp->getr();

                    randomizedX += qrand()/(double)RAND_MAX * dx - dx/2;
                    randomizedY += qrand()/(double)RAND_MAX * dy - dy/2;

                    a->setPosition(randomizedX, randomizedY, 0);
                    moveAgent(a);
                }
            }

            if (a->destination->gettype() == Ped::Twaypoint::TYPE_BIRTH)
            {
                // get the agents first ever waypoint and move it there to start over
                Ped::Twaypoint* wp = a->death_waypoint;
                Ped::Twaypoint* wc = a->birth_waypoint;

                double wx = wc->getx();
                double wy = wc->gety();
                
                if (sqrt( pow(ax-wx, 2.0) + pow(ay-wy, 2.0) ) <= ((wc->getr()))) 
                {
                // if (a->hasreacheddestination) {
                    double randomizedX = wp->getx();
                    double randomizedY = wp->gety();
                    double dx = wp->getr();
                    double dy = wp->getr();

                    randomizedX += qrand()/(double)RAND_MAX * dx - dx/2;
                    randomizedY += qrand()/(double)RAND_MAX * dy - dy/2;

                    a->setPosition(randomizedX, randomizedY, 0);
                    moveAgent(a);
                }
            }

        }   
  
    }
}

std::set<const Ped::Tagent*> Scene::getNeighbors(double x, double y, double maxDist)
{
    std::set<const Ped::Tagent*> potentialNeighbours = Ped::Tscene::getNeighbors(x, y, maxDist);

    // filter according to euclidean distance
    std::set<const Ped::Tagent*>::const_iterator agentIter = potentialNeighbours.begin();
    while(agentIter != potentialNeighbours.end())
    {
        double aX = (*agentIter)->getx();
        double aY = (*agentIter)->gety();
        double distance = sqrt((x-aX)*(x-aX) + (y-aY)*(y-aY));

        // remove distant neighbors
        if(distance > maxDist) 
        {
            potentialNeighbours.erase(agentIter++);
        } 
        else 
        {
            ++agentIter;
        }
    }

    return potentialNeighbours;
}


bool Scene::readFromFile(const QString& filename) 
{
    // open file
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) 
    {
        ROS_WARN("Couldn't open scenario file!");
        return false;
    }

    // read input
    while(!file.atEnd()) 
    {
        QByteArray line = file.readLine();
        processData(line);
    }

    // report success
    return true;
}


/// Called for each line in the file
void Scene::processData(QByteArray& data) 
{
    // TODO - switch to tinyxml for reading in the scene files

    xmlReader.addData(data);

    while(!xmlReader.atEnd()) 
    {
        xmlReader.readNext();
        if(xmlReader.isStartElement()) 
        {
            // start reading elements
            if ((xmlReader.name() == "scenario") || (xmlReader.name() == "welcome")) 
            {
                // nothing to do
                ROS_INFO("Starting to read elements");
            }
            else if (xmlReader.name() == "obstacle") 
            {
                double x1 = xmlReader.attributes().value("x1").toString().toDouble();
                double y1 = xmlReader.attributes().value("y1").toString().toDouble();
                double x2 = xmlReader.attributes().value("x2").toString().toDouble();
                double y2 = xmlReader.attributes().value("y2").toString().toDouble();
                Obstacle* obs = new Obstacle(x1, y1, x2, y2);
                this->addObstacle(obs);

                // fill the obstacle cells
                drawObstacles(x1, y1, x2, y2);
            }
            else if (xmlReader.name() == "waypoint") 
            {
                // TODO - add an explicit waypoint type
                QString id = xmlReader.attributes().value("id").toString();
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                double r = xmlReader.attributes().value("r").toString().toDouble();

                Waypoint* w = new Waypoint(id, x, y, r);

                if (boost::starts_with(id, "start")) 
                {
                    w->setType(Ped::Twaypoint::TYPE_BIRTH);
                    ROS_INFO("adding a birth waypoint");
                }

                if (boost::starts_with(id, "stop")) 
                {
                    w->setType(Ped::Twaypoint::TYPE_DEATH);
                    ROS_INFO("adding a death waypoint");
                }

                this->waypoints[id] = w;
            }
            else if (xmlReader.name() == "agent") 
            {
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                int n = xmlReader.attributes().value("n").toString().toDouble();
                double dx = xmlReader.attributes().value("dx").toString().toDouble();
                double dy = xmlReader.attributes().value("dy").toString().toDouble();
                double type = xmlReader.attributes().value("type").toString().toInt();
                for (int i=0; i<n; i++) 
                {
                    Agent* a = new Agent();

                    double randomizedX = x;
                    double randomizedY = y;
                    // handle dx=0 or dy=0 cases
                    if(dx != 0)
                        randomizedX += rand()/(double)RAND_MAX * dx - dx/2;
                    if(dy != 0)
                        randomizedY += rand()/(double)RAND_MAX * dy - dy/2;
                    a->setPosition(randomizedX, randomizedY);
                    a->setType(type);
                    this->addAgent(a);
                    currentAgents.append(a);
                }
            }
            else if (xmlReader.name() == "addwaypoint") 
            {
                QString id = xmlReader.attributes().value("id").toString();
                // add waypoints to current agents, not just those of the current <agent> element
                BOOST_FOREACH(Agent* a, currentAgents)
                {
                    a->addWaypoint(this->waypoints[id]);
                }
            }
            else 
            {
                // inform the user about invalid elements
                ROS_WARN("Unknown element");
            }
        }
        else if(xmlReader.isEndElement()) 
        {
            if (xmlReader.name() == "agent") 
            {
                currentAgents.clear();
            }
        }
    }

}



void Scene::drawObstacles(float x1, float y1, float x2, float y2)
{
    int i;               // loop counter
    int ystep, xstep;    // the step on y and x axis
    int error;           // the error accumulated during the increment
    int errorprev;       // *vision the previous value of the error variable
    int y = y1, x = x1;  // the line points
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;
    double unit_x,unit_y;
    unit_x=1;
    unit_y=1;
    // POINT (y1, x1);  // first point
    // NB the last point can't be here, because of its previous point (which has to be verified)
    if (dy < 0)
    {
        ystep = -unit_y;
        dy = -dy;
    }
    else
    {
        ystep = unit_y;
    }
    if (dx < 0)
    {
        xstep = -unit_x;
        dx = -dx;
    }
    else
    {
        xstep = unit_x;
    }

    ddy = 2 * dy;  // work with double values for full precision
    ddx = 2 * dx;
    
    obstacle_cells_.push_back(TLoc(x1,y1));

    if (ddx >= ddy)
    {  // first octant (0 <= slope <= 1)
        // compulsory initialization (even for errorprev, needed when dx==dy)
        errorprev = error = dx;  // start in the middle of the square
        for (i=0 ; i < dx ; i++)
        {  // do not use the first point (already done)
            x += xstep;
            error += ddy;
            if (error > ddx)
            {  // increment y if AFTER the middle ( > )
                y += ystep;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + errorprev < ddx)
                {  // bottom square also
                    obstacle_cells_.push_back(TLoc(x,y-ystep));
                }
                else if (error + errorprev > ddx)
                {
                    // left square also
                    obstacle_cells_.push_back(TLoc(x-xstep,y));
                }
                else
                {  // corner: bottom and left squares also
                    obstacle_cells_.push_back(TLoc(x,y-ystep));
                    obstacle_cells_.push_back(TLoc(x-xstep,y));

                }
            }
            obstacle_cells_.push_back(TLoc(x,y));
            errorprev = error;
        }
    }
    else
    {  // the same as above
        errorprev = error = dy;
        for (i=0 ; i < dy ; i++)
        {
            y += ystep;
            error += ddx;
            if (error > ddy)
            {
                x += xstep;
                error -= ddy;
                if (error + errorprev < ddy)
                {
                    obstacle_cells_.push_back(TLoc(x-xstep,y));
                }
                else if (error + errorprev > ddy)
                {
                    obstacle_cells_.push_back(TLoc(x,y-ystep));
                }
                else
                {
                    obstacle_cells_.push_back(TLoc(x-xstep,y));
                    obstacle_cells_.push_back(TLoc(x,y-ystep));
                }
            }

            obstacle_cells_.push_back(TLoc(x,y));
            errorprev = error;
        }
    }

    // ROS_INFO("loaded %d obstacle cells", (int)obstacle_cells_.size());
}


double* Scene::angleToQuaternion(double theta)
{
    double* q = new double[4];
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 1.0;


    // using namespace boost::numeric::ublas;
    // matrix<double> m (3, 3);
    // m(0,0) = cos(theta); m(0,1) = -sin(theta); m(0,2) = 0.0;
    // m(1,0) = sin(theta); m(1,1) = cos(theta); m(1,2) = 0.0;
    // m(2,0) = 0.0; m(2,1) = 0.0; m(2,2) = 1.0;

    // float trace = m(0,0) + m(1,1) + m(2,2); 
    // if( trace > 0 ) 
    // {
    //     float s = 0.5f / sqrtf(trace+ 1.0f);
    //     q[3] = 0.25f / s;
    //     q[2] = ( m(2,1) - m(1,2) ) * s;
    //     q[1] = ( m(0,2) - m(2,0) ) * s;
    //     q[0] = ( m(1,0) - m(0,1) ) * s;
    // } 
    // else 
    // {
    //     if ( m(0,0) > m(1,1) && m(0,0) > m(2,2) ) 
    //     {
    //         float s = 2.0f * sqrtf( 1.0f + m(0,0) - m(1,1) - m(2,2));
    //         q[3] = (m(2,1) - m(1,2) ) / s;
    //         q[2] = 0.25f * s;
    //         q[1] = (m(0,1) + m(1,0) ) / s;
    //         q[0] = (m(0,2) + m(2,0) ) / s;
    //     } 
    //     else if (m(1,1) > m(2,2)) 
    //     {
    //         float s = 2.0f * sqrtf( 1.0f + m(1,1) - m(0,0) - m(2,2));
    //         q[3] = (m(0,2) - m(2,0) ) / s;
    //         q[2] = (m(0,1) + m(1,0) ) / s;
    //         q[1] = 0.25f * s;
    //         q[0] = (m(1,2) + m(2,1) ) / s;
    //     } 
    //     else 
    //     {
    //         float s = 2.0f * sqrtf( 1.0f + m(2,2) - m(0,0) - m(1,1) );
    //         q[3] = (m(1,0) - m(0,1) ) / s;
    //         q[2] = (m(0,2) + m(2,0) ) / s;
    //         q[1] = (m(1,2) + m(2,1) ) / s;
    //         q[0] = 0.25f * s;
    //     }
    // }

    return q;
}




int main(int argc, char** argv)
{
    // initialize resources
    ros::init(argc, argv, "simulator");

    ROS_INFO("node initialized");

    ros::NodeHandle node;

    double x1,x2, y1,y2;
    node.getParam("/irl_features/x1", x1);
    node.getParam("/irl_features/x2", x2);
    node.getParam("/irl_features/y1", y1);
    node.getParam("/irl_features/y2", y2);

    // Scene sim_scene(0, 0, 45, 45, node);
    // Scene sim_scene(0, 0, 300, 100, node);
    Scene sim_scene(x1, y1, x2, y2, node);

    if (sim_scene.initialize()) 
    {
        ROS_INFO("loaded parameters, starting simulation...");
        sim_scene.runSimulation();
    }
    else
        return EXIT_FAILURE;

    return EXIT_SUCCESS;    
}

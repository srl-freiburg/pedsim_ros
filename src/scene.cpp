
#include "scene.h"

Scene::Scene(const ros::NodeHandle& node)  
    : Ped::Tscene(), nh_(node)
{
    // start the time steps
    timestep = 0;

    // useful for keeping track of agents in the cleaning process
    tree = new Ped::Ttree(this, 0, 0, 0, 5000, 5000);

    /// setup the list of all agents and the robot agent
    all_agents_.clear();

    // setup services and publishers
    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState>("AllAgentsStatus", 0);
    pub_agent_visuals_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    srv_move_agent_ = nh_.advertiseService("SetAgentState", &Scene::srvMoveAgentHandler, this);
}




bool Scene::srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request& req, pedsim_srvs::SetAgentState::Response& res)
{
    pedsim_msgs::AgentState state = req.state;

    ROS_INFO("Rceived (%f) (%f)", state.position.x, state.position.y);

    if (robot_->getid() == state.id)  {
        robot_->setvx(state.velocity.x);
        robot_->setvy(state.velocity.y);
    }

    res.finished = true;

    return true;
}



void Scene::cleanupItems() {
    cleanup();
}

void Scene::clear() {
    all_agents_.clear();

    foreach(Waypoint* waypoint, waypoints)
        delete waypoint;
    waypoints.clear();

    foreach(Obstacle* obs, obstacles)
        delete obs;
    obstacles.clear();
}

void Scene::runSimulation() {

    /// setup the agents and the robot
    all_agents_ = getAllAgents();

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);

        if (a->gettype() == 2)
            robot_ = a;
    }

    ros::Rate r(20);

    while (ros::ok())
    {
        moveAllAgents();

        publicAgentStatus();
        publishAgentVisuals();

        // helps to make things faster
        cleanupItems();

        ros::spinOnce();

        r.sleep();
    }
}


void Scene::moveAllAgents()
{
    timestep++;

    // move the agents by social force
    Ped::Tscene::moveAgents(CONFIG.simh);
}



void Scene::publicAgentStatus()
{
    pedsim_msgs::AllAgentsState all_status;
    all_status.header.stamp = ros::Time::now();

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        pedsim_msgs::AgentState state;

        state.header.stamp = ros::Time::now();
        state.id = a->getid();
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

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "pedsim_base";
        marker.header.stamp = ros::Time();
        marker.ns = "pedsim";
        marker.id = a->getid();

        if (a->gettype() == robot_->gettype()) 
        {
            // marker.type = visualization_msgs::Marker::CUBE;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://simulator/images/darylbot.dae";
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
        }
        else
        {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 1;
        }

        marker.action = 0;  // add or modify
        marker.pose.position.x = a->getx();
        marker.pose.position.y = a->gety();
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        pub_agent_visuals_.publish( marker );

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
        if(distance > maxDist) {
            potentialNeighbours.erase(agentIter++);
        } else {
            ++agentIter;
        }
    }

    return potentialNeighbours;
}


bool Scene::readFromFile(const QString& filename) {
    // ROS_INFO("Loading scenario file '%s'.", filename.toStdString);

    // open file
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        // ERROR_LOG("Couldn't open scenario file!");
        return false;
    }

    // read input
    while(!file.atEnd()) {
        QByteArray line = file.readLine();
        processData(line);
    }

    // report success
    return true;
}


/// Called for each line in the file
void Scene::processData(QByteArray& data) {
    xmlReader.addData(data);
    CONFIG.obstacle_positions.clear();

    while(!xmlReader.atEnd()) {
        xmlReader.readNext();
        if(xmlReader.isStartElement()) {
            if((xmlReader.name() == "scenario")
                    || (xmlReader.name() == "welcome")) {
                // nothing to do
            }
            else if(xmlReader.name() == "obstacle") {
                double x1 = xmlReader.attributes().value("x1").toString().toDouble();
                double y1 = xmlReader.attributes().value("y1").toString().toDouble();
                double x2 = xmlReader.attributes().value("x2").toString().toDouble();
                double y2 = xmlReader.attributes().value("y2").toString().toDouble();
                Obstacle* obs = new Obstacle(x1, y1, x2, y2);
                this->addObstacle(obs);
                // drawObstacles(x1, y1, x2, y2);
            }
            else if(xmlReader.name() == "waypoint") {
                // TODO - add an explicit waypoint type
                QString id = xmlReader.attributes().value("id").toString();
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                double r = xmlReader.attributes().value("r").toString().toDouble();

                Waypoint* w = new Waypoint(id, x, y, r);

                if (boost::starts_with(id, "start")) {
                    w->setType(Ped::Twaypoint::TYPE_BIRTH);
                    std::cout << "adding a birth waypoint" << std::endl;
                }

                if (boost::starts_with(id, "stop")) {
                    w->setType(Ped::Twaypoint::TYPE_DEATH);
                    std::cout << "adding a death waypoint" << std::endl;
                }

                this->waypoints[id] = w;
            }
            else if(xmlReader.name() == "agent") {
                double x = xmlReader.attributes().value("x").toString().toDouble();
                double y = xmlReader.attributes().value("y").toString().toDouble();
                int n = xmlReader.attributes().value("n").toString().toDouble();
                double dx = xmlReader.attributes().value("dx").toString().toDouble();
                double dy = xmlReader.attributes().value("dy").toString().toDouble();
                double type = xmlReader.attributes().value("type").toString().toInt();
                //TODO: keep agent group and expand later!?
                for (int i=0; i<n; i++) {
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
            else if(xmlReader.name() == "addwaypoint") {
                QString id = xmlReader.attributes().value("id").toString();
                // add waypoints to current agents, not just those of the current <agent> element
                foreach(Agent* a, currentAgents)
                    a->addWaypoint(this->waypoints[id]);
            }
            else {
                // inform the user about invalid elements
                // ROS_WARN("Unknown element: %s", xmlReader.name().toString());
            }
        }
        else if(xmlReader.isEndElement()) {
            if (xmlReader.name() == "agent") {
                currentAgents.clear();
            }
        }
    }

}


void Scene::drawObstacles(float x1, float y1, float x2, float y2)
{
    // Modified Bresenham's line algorithm (addding a buffer around obstacles)
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 8.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            CONFIG.obstacle_positions.push_back(TLoc(y,x));
            CONFIG.obstacle_positions.push_back(TLoc(y+CONFIG.height,x+CONFIG.width));
            CONFIG.obstacle_positions.push_back(TLoc(y-CONFIG.height,x-CONFIG.width));
        }
        else
        {
            CONFIG.obstacle_positions.push_back(TLoc(x,y));
            CONFIG.obstacle_positions.push_back(TLoc(x+CONFIG.width,y+CONFIG.height));
            CONFIG.obstacle_positions.push_back(TLoc(x-CONFIG.width,y-CONFIG.height));
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}


int main(int argc, char** argv)
{
    // initialize resources
    ros::init(argc, argv, "simulator");

    ROS_INFO("node initialized");

    ros::NodeHandle node;

    Scene sim_scene(node);

    ROS_INFO("Simulation scene started");

    // load parameters
    std::string scene_file_param;
    node.getParam("/simulator/scene_file", scene_file_param);

    double cell_size;
    node.getParam("/simulator/cell_size", cell_size);
    CONFIG.width = cell_size;
    CONFIG.height = cell_size;

    // load scenario file
    QString scenefile = QString::fromStdString(scene_file_param);

    if (!sim_scene.readFromFile(scenefile)) {
        ROS_WARN("Could not load the scene file, check paths");
    }


    ROS_INFO("loaded parameters");
    sim_scene.runSimulation();

    // ScenarioReader scenarioReader(scene);

    return 0;    
}


#include <scene.h>

/// global to keep persistence over instances of scene
Grid* Scene::grid_ = NULL;


Scene::Scene(QGraphicsScene* guiSceneIn, const ros::NodeHandle& node)  
    : nh_(node)
{
    guiScene = guiSceneIn;

    if (CONFIG.scenario == AIRPORT) {
        // QRect area(0, 0, 200+CONFIG.width, 200+CONFIG.height); // grid test
         QRect area(0, 0, 820, 820); // grid test large
//        QRect area(0, 0, 820, 200); // grid test hard

        Scene::grid_ = new Grid(area.x(), area.y(), area.width(), area.height(), guiScene);
        tree = new Ped::Ttree(this, 0, area.x(), area.y(), area.width(), area.height());

        // QRect area(0, 0, 300+CONFIG.width, 150+CONFIG.height); // airport gate
        // Scene::grid_ = new Grid(area.x(), area.y(), area.width(), area.height(), guiScene);
        // tree = new Ped::Ttree(this, 0, area.x(), area.y(), area.width(), area.height());
    } else {
        QRect area(0, 0, 300, 100); // henri experiment
        Scene::grid_ = new Grid(area.x(), area.y(), area.width(), area.height(), guiScene);
        tree = new Ped::Ttree(this, 0, area.x(), area.y(), area.width(), area.height());
    }
    QObject::connect(&movetimer, SIGNAL(timeout()), this, SLOT(moveAllAgents()));
    movetimer.setInterval(500);

    QObject::connect(&cleanuptimer, SIGNAL(timeout()), this, SLOT(cleanupSlot()));
    cleanuptimer.setInterval(200);

    connect(&CONFIG, SIGNAL(waypointVisibilityChanged(bool)), this, SLOT(updateWaypointVisibility(bool)));

    // start the time steps
    timestep = 0;

    /// setup the list of all agents and the robot agent
    all_agents_.clear();
    all_agents_ = getAllAgents();


    // setup services and publishers
    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState>("AllAgentsStatus", 1);
    srv_move_agent_ = nh_.advertiseService("SetAgentState", &Scene::srvMoveAgentHandler, this);

    // additional initialization in separat methods to keep constructor clean
    initializeAll();
    unpauseUpdates();
}


Scene::~Scene() { clear(); }
Grid* Scene::getGrid() { return grid_; }


bool Scene::srvMoveAgentHandler(pedsim_srvs::SetAgentState::Request& req, pedsim_srvs::SetAgentState::Response& res)
{
    pedsim_msgs::AgentState state = req.state;

    if (robot_->getid() == state.id)  {
        robot_->setPosition(state.position.x*20.0, state.position.y*20.0, state.position.z*20.0 );
        // robot_->setPosition(state.position.y*20.0, state.position.x*20.0, state.position.z*20.0 );
        robot_->setvx(state.velocity.x);
        robot_->setvy(state.velocity.y);

        guiScene->addEllipse(robot_->getx(), robot_->gety(), 0.5, 0.5, QPen(Qt::yellow, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin), QBrush(Qt::yellow));

        moveAgent(robot_);
    }

    res.finished = true;

    return true;
}


void Scene::initializeAll()
{
    // mark obstacle cells
    for (size_t i = 0; i < CONFIG.obstacle_positions.size(); i++) {
        TLoc l = CONFIG.obstacle_positions[i];
        getGrid()->setOccupied(l.x, l.y);
    }
}

bool Scene::isPaused() const
{
    return !movetimer.isActive();
}

void Scene::pauseUpdates()
{
    movetimer.stop();
    cleanuptimer.stop();
}

void Scene::unpauseUpdates()
{
    movetimer.start();
    cleanuptimer.start();
}


void Scene::cleanupSlot() {
    cleanup();
}

void Scene::clear() {
    foreach(Agent* agent, agents)
        delete agent;
    agents.clear();
    all_agents_.clear();

    foreach(Waypoint* waypoint, waypoints)
        delete waypoint;
    waypoints.clear();

    foreach(Obstacle* obs, obstacles)
        delete obs;
    obstacles.clear();
}

void Scene::updateWaypointVisibility(bool visible) {
    foreach(Waypoint* waypoint, waypoints)
        waypoint->setVisible(visible);
}



void Scene::moveAllAgents() {
    if(!movetimer.isActive()) return;

    all_agents_ = getAllAgents();

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) 
    {
        Ped::Tagent *a = (*iter);

        if (a->gettype() == 2) 
            robot_ = a;
    }

    publicAgentStatus();

    /// NOTE Test Termination criterion for the robot
    if (eDist(robot_->getx(), robot_->gety(), 100, 720) < 1.0) exit(0);

    timestep++;
    ros::spinOnce();

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
        state.position.x = a->getx() * (1/20.0);
        state.position.y = a->gety() * (1/20.0);
        state.position.z = a->getz() * (1/20.0);

        state.velocity.x = a->getvx();
        state.velocity.y = a->getvy();
        state.velocity.z = a->getvz();

        all_status.agent_states.push_back(state);

    }

    pub_all_agents_.publish(all_status);
}



std::set<const Ped::Tagent*> Scene::getNeighbors(double x, double y, double maxDist)
{
    std::set<const Ped::Tagent*> potentialNeighbours = Ped::Tscene::getNeighbors(x, y, maxDist);

    // filter according to euclidean distance
    auto agentIter = potentialNeighbours.begin();
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




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
    movetimer.setInterval(50);

    QObject::connect(&cleanuptimer, SIGNAL(timeout()), this, SLOT(cleanupSlot()));
    cleanuptimer.setInterval(200);

    QObject::connect(&speed_timer, SIGNAL(timeout()), this, SLOT(changeSpeeds()));
    speed_timer.setInterval(50);

    QObject::connect(&teleop_timer, SIGNAL(timeout()), this, SLOT(teleopControl()));
    teleop_timer.setInterval(50);

    QObject::connect(&robot_timer, SIGNAL(timeout()), this, SLOT(driveWithPolicy()));
    robot_timer.setInterval(50);

    QObject::connect(&log_timer, SIGNAL(timeout()), this, SLOT(logExperimentData()));
    log_timer.setInterval(50);

    connect(&CONFIG, SIGNAL(waypointVisibilityChanged(bool)), this, SLOT(updateWaypointVisibility(bool)));

    // start the time steps
    timestep = 0;

    // setup distribution sampling
    sampler_ = new Sampler(1.8, 0.26); // based on the helbing paper
    // sampler_ = new Sampler(2.68, 0.26); // based on the helbing paper

    /// setup the list of all agents and the robot agent
    /// also set the inital random agent velocities
    all_agents_.clear();
    all_agents_ = getAllAgents();


    pub_all_agents_ = nh_.advertise<pedsim_msgs::AllAgentsState>("AllAgentsStatus", 1);

    // additional initialization in separat methods to keep constructor clean
    initializeAll();

    unpauseUpdates();
}


Scene::~Scene() { clear(); }
Grid* Scene::getGrid() { return grid_; }


void Scene::initializeAll()
{
    // load the learned policy (weights)
    icra_policy_ = readICRAPolicyFromFile(CONFIG.weight_file);

    // mark obstacle cells
    for (size_t i = 0; i < CONFIG.obstacle_positions.size(); i++) {
        TLoc l = CONFIG.obstacle_positions[i];
        getGrid()->setOccupied(l.x, l.y);
    }

    // setup goals, waypoints and trajectories
    if (CONFIG.scenario == AIRPORT) {
        goal_ = Tpoint(280, 100, 0);  // airport scene
        start_ = Tpoint(20, 130, 0);
    } else {
        goal_ = Tpoint(200, 90, 0);  // Henri experiment
        start_ = Tpoint(20, 90, 0);
    }

    old_wp_ = start_;
    next_wp_ = Tpoint(start_.x+CONFIG.width, start_.y);
    pwp = start_;
    traj_step_ = 0;
    trajectory_ = new Trajectory();
    lcc_ = start_;

    // setup faeture processing
    ficra_ = new CFeatures_ICRA();

    //setup graph search
    // graph_search_.reset();
    // graph_search_ = boost::shared_ptr<GraphSearch>(new GraphSearch());
    // graph_search_->createGraphFromGrid(getGrid());

    // initialize metrics
    anisotropics_.towards = 0;
    anisotropics_.orthogonal = 0;
    anisotropics_.behind = 0;

    proxemics_.intimate = 0;
    proxemics_.personal = 0;
    proxemics_.social = 0;
    proxemics_.publik = 0;

    closest_distance_ = 100000.0;

    /// Generate the initial path (and show it, it is not weighted)
    // generateNewTrajectory(true);
}

/// =========================================================================
/// Simulation star/stop control
/// =========================================================================
bool Scene::isPaused() const
{
    return !movetimer.isActive();
}

void Scene::pauseUpdates()
{
    movetimer.stop();
    cleanuptimer.stop();
    speed_timer.stop();
    log_timer.stop();

    if (CONFIG.drive_mode == TELEOP)
        teleop_timer.stop();
    if (CONFIG.drive_mode == FOLLOW_POLICY)
        robot_timer.stop();
}

void Scene::unpauseUpdates()
{
    movetimer.start();
    cleanuptimer.start();
    //speed_timer.start();
    // log_timer.start();
}


/// =========================================================================
/// Periodically clear the scene to speed up things and manipulate waypoints
/// =========================================================================
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


/// =========================================================================
/// Periodically change the max speed of agents
/// =========================================================================
void Scene::changeSpeeds()
{
    if (timestep < 5) {
        for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
            Ped::Tagent *a = (*iter);

            if (a->gettype() == robot_->gettype()) continue;

            double speed = sampler_->sampleFromNormalDistribution();
            a->setVmax(speed);
        }
    }
}

void Scene::moveAllAgents() {
    if(!movetimer.isActive()) return;

    all_agents_ = getAllAgents();

    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        if (a->gettype() == 1)  {
            robot_ = a;
        }

    }
    
    /// BEGIN Smoke tests
//    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
//        Ped::Tagent *a = (*iter);
//        if (a->gettype() == 1)  {
//            robot_ = a;
//        }
//        std::cout << ( timestep ) << " ";
//        std::cout << ( timestep ) * (1/200.0) << " ";
//        std::cout << a->getid() << " ";
//        std::cout << a->getx() * (1/20.0) << " ";
//        std::cout << a->gety() * (1/20.0) << " ";
//        std::cout << a->gettype() << "\n";
//    }

     // simpleTestCostMap();
    // CONFIG.sensor_horizon = (20.0)* CONFIG.sensor_horizon;


    // smokeFS();
    publicAgentStatus();

    /// Termination criterion for the robot
    if (eDist(robot_->getx(), robot_->gety(), 100, 720) < 1.0) exit(0);

    //    if (eDist(robot_->getx(), robot_->gety(), 200, 60) < 1.0) exit(0);

    timestep++;

    // move the agents by social force
    Ped::Tscene::moveAgents(CONFIG.simh);
}



void Scene::publicAgentStatus()
{
    pedsim_msgs::AllAgentsState all_status;
    
    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        pedsim_msgs::AgentState state;
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


void Scene::spawnKillAgents() 
{
    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);
  
        if (a->gettype() != 2 && timestep > 10) {
            if (a->destination->gettype() == Ped::Twaypoint::TYPE_DEATH)
            {
                // get the agents first ever waypoint and move it there to start over
                Ped::Twaypoint* wp = a->birth_waypoint;
                Ped::Twaypoint* wc = a->death_waypoint;

                if (eDist(a->getx(), a->gety(), wc->getx(), wc->gety()) <= ((wc->getr() + 2) / 20.0) ) {
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
                
                if (eDist(a->getx(), a->gety(), wc->getx(), wc->gety()) <= ((wc->getr() + 2) / 20.0)) {
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


/// =========================================================================
/// Simple tests on the simulator scaling
/// =========================================================================
void Scene::smokeFS() 
{
    Ped::Tvector dir(1,1,0);

    // compute feature for the interest node 20,5
//    Ped::Tvector cell(400, 100, 0);
    Ped::Tvector cell(400, 400, 0);
    FeatureICRA f = ficra_->computeCellFeaturesFS5(all_agents_, robot_, cell.x, cell.y, CONFIG.sensor_horizon);
    std::cout << timestep << " ";

//    std::cout << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << " ";
    std::cout << f[0] << " " << f[1] << " ";
    std::cout << ficra_->computeFeaturesCost(f, icra_policy_) << " ";


    // store the costmap locally for testing
    boost::numeric::ublas::matrix<double> costmap_sum(getGrid()->width / CONFIG.width, getGrid()->height / CONFIG.height);
    float sum_all = 0.0;
    float avr_all = 0.0;

    for (unsigned i = 0; i < costmap_sum.size1(); ++i) {
        for (unsigned j = 0; j < costmap_sum.size2(); ++j) {
            // compute feature over the entire costmap
            FeatureICRA fc = ficra_->computeCellFeaturesFS5(all_agents_, robot_, i * CONFIG.width, j * CONFIG.height, CONFIG.sensor_horizon);
            double fcost = ficra_->computeFeaturesCost(fc, icra_policy_);

            costmap_sum(i, j) = fcost;
            sum_all += fcost;
        }
    }

//    avr_all = sum_all / (costmap_sum.size1() * costmap_sum.size2() * 8);
    avr_all = sum_all / (costmap_sum.size1() * costmap_sum.size2() );
    std::cout << sum_all << " " << avr_all << std::endl;
}


std::array<float, 5> Scene::smokeFeature()
{
    float cdistance = 100000.0;
    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        if (a->gettype() != robot_->gettype()) 
        {
            float dist = eDist(a->getx(), a->gety(), robot_->getx(), robot_->gety());
            if (dist < cdistance)
                cdistance = dist;
        }
    }

    // feature: d, vx, vy, x, y
    std::array<float, 5> sf; sf.fill(0.0);
    sf[0] = cdistance;
    sf[1] = robot_->getvx();
    sf[2] = robot_->getvy();
    sf[3] = robot_->getx() * (1/20.0);
    sf[4] = robot_->gety() * (1/20.0);

    return sf;
}

float Scene::computeSmokeCost(const std::array<float, 5> f)
{
    float cost = 0.0;
    float N = 5;
    std::array<float, 5>  w; w.fill(0);

    w[0] = 1 / N;
    w[1] = 1 / (N-1);
    w[2] = 1 / (N-2);
    w[3] = 1 / (N-3);
    w[4] = 1 / (N-4);

    for (size_t i = 0; i < 5; i++) {
        cost += w[i] * f[i];
    }

    return cost;
}

void Scene::simpleTestCostMap()
{
    // boost::numeric::ublas::matrix<double> costmap_avr(getGrid()->width / CONFIG.width, getGrid()->height / CONFIG.height);
    boost::numeric::ublas::matrix<double> costmap_sum(getGrid()->width / CONFIG.width, getGrid()->height / CONFIG.height);

    std::vector<Ped::Tvector> directions;
    directions.push_back(Ped::Tvector(1, 0, 0));
    directions.push_back(Ped::Tvector(1, 1, 0));
    directions.push_back(Ped::Tvector(0, 1, 0));
    directions.push_back(Ped::Tvector(-1, 1, 0));
    directions.push_back(Ped::Tvector(-1, 0, 0));
    directions.push_back(Ped::Tvector(-1, -1, 0));
    directions.push_back(Ped::Tvector(0, -1, 0));
    directions.push_back(Ped::Tvector(1, -1, 0));

    float sum_all = 0.0;
    float avr_all = 0.0;


    /// checking the feature for cell [0,0]
    // foreach(Ped::Tvector dir, directions) {

    Ped::Tvector dir(1,1,0);
        std::array<float, 5> sf; sf.fill(0.0);
        // sf[0] = getClosestDistance(Tpoint(i*CONFIG.width, j*CONFIG.height, 0.0));
        // sf[0] = getClosestDistance(Tpoint(400, 400, 0.0));
        sf[0] = getClosestDistance(Tpoint(400, 100, 0.0));

        double theta = atan2(dir.y, dir.x);
        sf[1] = CONFIG.robot_speed * cos(theta);
        sf[2] = CONFIG.robot_speed * sin(theta);
        sf[3] = 400  * (1/20.0);
        // sf[4] = 400  * (1/20.0);
        sf[4] = 100  * (1/20.0);
        double cost = computeSmokeCost(sf);
        double factor = sqrt((dir.x*dir.x) + (dir.y*dir.y));

        
        cout << timestep << " ";
        cout << sf[0] << " ";
        cout << sf[1] << " ";
        cout << sf[2] << " ";
        cout << sf[3] << " ";
        cout << sf[4] << " ";
        cout << factor*cost << " ";


    for (unsigned i = 0; i < costmap_sum.size1(); ++i) {
        for (unsigned j = 0; j < costmap_sum.size2(); ++j) {
            // cost in the 8 directions (summed up)
            float cost = 0.0;
            foreach(Ped::Tvector dir, directions) {
                // Ped::Tvector dir(1,1,0);
                double factor = sqrt((dir.x*dir.x) + (dir.y*dir.y));

                std::array<float, 5> sf; sf.fill(0.0);
                sf[0] = getClosestDistance(Tpoint(i*CONFIG.width, j*CONFIG.height, 0.0));
                double theta = atan2(dir.y, dir.x);
                sf[1] = CONFIG.robot_speed * cos(theta);
                sf[2] = CONFIG.robot_speed * sin(theta);
                sf[3] = i * CONFIG.width * (1/20.0);
                sf[4] = j * CONFIG.height * (1/20.0);
                cost += factor*computeSmokeCost(sf);
            }

            costmap_sum(i, j) = cost;
            sum_all += cost;
        }
    }

    avr_all = sum_all / (costmap_sum.size1() * costmap_sum.size2() * 8);
    // avr_all = sum_all / (costmap_sum.size1() * costmap_sum.size2() * 1);

    // // cout << "Size: " << costmap_sum.size1() << " " << costmap_sum.size2() << endl;
    std::cout << sum_all << " " << avr_all << std::endl;
    // std::cout << timestep << " " << sum_all << " " << avr_all << std::endl;
}


float Scene::getClosestDistance(Tpoint p) 
{
    float cdistance = 100000.0;
    for (vector<Ped::Tagent*>::const_iterator iter = all_agents_.begin(); iter != all_agents_.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        if (a->gettype() == robot_->gettype()) continue;

        float dist = eDist(a->getx(), a->gety(), p.x+(CONFIG.width/2.0), p.y+(CONFIG.height/2.0));
        if (dist < cdistance)
            cdistance = dist;

    }
    return cdistance;
}

/// =========================================================================
/// Visualize a path (color the cells on path)
/// =========================================================================
void Scene::visualizePath(std::vector<GridCell> path, QColor col) 
{
    QPen pen(col, 0.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    double xl = path[0].x, yl = path[0].y;

    for (size_t i = 1; i < path.size(); i++) {
        guiScene->addLine(xl, yl, path[i].x, path[i].y, pen);
        xl = path[i].x;
        yl = path[i].y;
    }
}


/// ============================================================================
/// Teleop
/// ============================================================================
void Scene::teleopControl() 
{
    // wait for dynamics to settle
    if (timestep < 10) return;

    if (CONFIG.drive_mode == TELEOP ) {
        double angle = ((10 - CONFIG.robot_direction) / 10.0)*180;
        double vx = cos(angle * M_PI/180.0);
        double vy = sin(angle * M_PI/180.0);

        double stepx = CONFIG.robot_speed * vx;
        double stepy = CONFIG.robot_speed * vy;
        robot_->setvx(stepx);
        robot_->setvy(stepy);
        robot_->setPosition(robot_->getx()+stepx, robot_->gety()+stepy, robot_->getz() );
        moveAgent(robot_);

        guiScene->addEllipse(robot_->getx(), robot_->gety(), 0.5, 0.5, QPen(Qt::green, 0.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin), QBrush(Qt::green));


        // updateFullCostMap();
        // visualizeCostMap(true);

        // todo: update  and log metrics
        checkGoalReached();
        updateMetrics();
    }

    std::cout << "TIMESTEP: " << timestep << std::endl;
}


/// ============================================================================
/// Drive the robot using select policy and configuration
/// ============================================================================
void Scene::driveWithPolicy()
{
    if (CONFIG.drive_mode == FOLLOW_POLICY)
    {
        if (CONFIG.run_algorithm == ASTAR) {
            driveSingleStep();
            checkGoalReached();
            checkWaypointReached();

            if (CONFIG.update_type == ORACLE)
                updateFullCostMap();
            else if (CONFIG.update_type == LOCAL)
                updateLocalCostMap();

            if (CONFIG.show_visualization)
                visualizeCostMap(true);

            // if (getAgentsInRange(all_agents_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon).size() > 0)
            //     generateNewTrajectory(false);

            if (pathIsBlocked(all_agents_, trajectory_->getPath()))
                generateNewTrajectory(false);


        } else {
            // driveSingleStep();
            driveSingleStepGradient();
            // checkWaypointReached();
            checkGoalReached();

            if (CONFIG.update_type == ORACLE)
                updateFullCostMap();
            else
                updateLocalCostMap();

            generateNewDijkstraCostMap();
        }
    }
}

// check if we reached the goal, if yes, terminate
void Scene::checkGoalReached() 
{
    if (( eDist(robot_->getx(), robot_->gety(), goal_.x, goal_.y) < CONFIG.touch_radius*3 ) 
        ||
        (robot_->getx() > goal_.x+(CONFIG.width*4) )) {
//        exit(0);    // for automation cases
        pauseUpdates();
    }
}

// check if the next waypoint is reached
void Scene::checkWaypointReached() {
    if ( eDist(robot_->getx(), robot_->gety(), next_wp_.x, next_wp_.y) < CONFIG.touch_radius ) {
        traj_step_ += 1;

        old_wp_ = next_wp_;

        if (traj_step_+1 < trajectory_->length())
            next_wp_ = trajectory_->getPath()[traj_step_];
        if (traj_step_+2 < trajectory_->length())
            next_wp_ = trajectory_->getPath()[traj_step_+2];
        if (traj_step_+3 < trajectory_->length())
            next_wp_ = trajectory_->getPath()[traj_step_+3];
    }
}

// drive one step along a trajectory
void Scene::driveSingleStep() {
    double vx = (next_wp_.x - old_wp_.x);
    double vy = (next_wp_.y - old_wp_.y);

    double theta = atan2(vy, vx);

    double stepx = (CONFIG.robot_speed) * cos(theta);
    double stepy = (CONFIG.robot_speed) * sin(theta);

    robot_->setPosition(robot_->getx()+stepx, robot_->gety()+stepy, robot_->getz());
    robot_->setvx(stepx);
    robot_->setvy(stepy);

    guiScene->addEllipse(robot_->getx(), robot_->gety(), 0.5, 0.5, QPen(Qt::green, 0.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin), QBrush(Qt::green));

    moveAgent(robot_);

    updateMetrics();
}

// drive one step on the gradient from Dijkstra costmap
void Scene::driveSingleStepGradient() {
    // gradients from costmap
    cv::Mat hgradient_ = computeGradient(getGrid(), dcosts_, 0);
    cv::Mat vgradient_ = computeGradient(getGrid(), dcosts_, 1);
    cv::Mat orig = computeGradient(getGrid(), dcosts_, 2);

    /// avoid these as they slow down the runs considerably
    // writeMatToFile(hgradient_, "hor.txt");
    // writeMatToFile(vgradient_, "ver.txt");
    // writeMatToFile(orig, "cost.txt");

    // get the current robot cell
    Tpoint ec = getEnclosingCell(getGrid(), robot_->getx(), robot_->gety());

    boost::numeric::ublas::matrix<double> hg = getCostMatrix(hgradient_);
    boost::numeric::ublas::matrix<double> vg = getCostMatrix(vgradient_);
    boost::numeric::ublas::matrix<double> cg = getCostMatrix(orig);

    // Tpoint nc = nextLowCell(ec, cg);
    Tpoint nc = getLowCellInRange(ec, CONFIG.sensor_horizon*2, cg, getGrid());

    // double theta = atan2((vg(nc.x, nc.y) - vg(ec.x, ec.y)), (hg(nc.x, nc.y) - hg(ec.x, ec.y)));
    double theta = atan2((vg(ec.x, ec.y) - vg(nc.x, nc.y)), (hg(ec.x, ec.y) - hg(nc.x, nc.y)));
    // double theta = atan2((ec.y - nc.y), (ec.x - nc.x));    
    // double theta = atan2((nc.y - ec.y), (nc.x - ec.x));    
    // double theta = atan2(vg(nc.x, nc.y), hg(nc.x, nc.y));

    double stepx = (CONFIG.robot_speed) * cos(theta);
    double stepy = (CONFIG.robot_speed) * sin(theta);

    // std::cout << theta << " " << stepx << " " << stepy << " | " << CONFIG.dmax << " " << CONFIG.dmin << endl;

    robot_->setPosition(robot_->getx()+stepx, robot_->gety()+stepy, robot_->getz());
    robot_->setvx(stepx);
    robot_->setvy(stepy);
    moveAgent(robot_);

    // updateMetrics();

    guiScene->addEllipse(robot_->getx(), robot_->gety(),0.5,0.5, QPen(Qt::green, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin), QBrush(Qt::green));
}

void Scene::driveSingleStepSocialForce() {
    // reverted to the internal library
}


/// ============================================================================
/// Costmap updates (full and local) and their visualization
/// ============================================================================
void Scene::updateFullCostMap()
{
    
}

void Scene::updateLocalCostMap()
{
    
}

void Scene::visualizeCostMap(bool skeleton)
{
    
}

/// =========================================================================
/// Path planning methods
/// =========================================================================
void Scene::generateNewTrajectory(bool initial) {
   // initial = true;
    if (initial == false) {
        std::vector<std::vector<GNode> > paths;
        Tpoint ec = getEnclosingCell(getGrid(), robot_->getx(), robot_->gety());
        generatePath(paths, GNode(ec.x, ec.y));

        if (paths.size() > 0 && paths[0].size() > 1) {
            convertPathToTrajectory(paths[0], true);

            // visualizePath2();
            
            if (CONFIG.show_visualization)
                visualizePath2();
        }
    }
    else {

        graph_search_.reset();
        graph_search_ = boost::shared_ptr<GraphSearch>(new GraphSearch());
        graph_search_->createGraphFromGrid(getGrid(), true, true);
        //        graph_search_->createGraphFromGrid(getGrid(), false, false);

        if (traj_step_ > 1) {
            Tpoint ec = getEnclosingCell(getGrid(), robot_->getx(), robot_->gety());
            pwp = Tpoint(ec.x*CONFIG.width, ec.y*CONFIG.height, 0);
        }
        std::vector<GridCell> path = graph_search_->computePathWeightedAStar(pwp, goal_);
        // if (initial)
            // visualizePath(path, Qt::white);

        // convert path to trajectory
        if (path.size() == 0) return;

        trajectory_->reset();
        traj_step_ = 0;
        for(int i = path.size()-1; i >= 0; i--)
            trajectory_->addPointEnd(Tpoint(path[i].x, path[i].y, 0));
    }
}

void Scene::generateNewDijkstraCostMap() {
    graph_search_.reset();
    graph_search_ = boost::shared_ptr<GraphSearch>(new GraphSearch());
    graph_search_->createGraphFromGrid(getGrid(), true, true);
    dcosts_ = graph_search_->computeAllPathsToGoal(getGrid(), goal_);

    // std::vector<GridCell> path = graph_search_->getSinglePathDijkstra(next_wp_, goal_);
    // convert path to trajectory
    // if (path.size() == 0) return;

    // visualizePath(path, Qt::white);

    // trajectory_->reset();
    // traj_step_ = 0;
    // for(int i = path.size()-1; i >= 0; i--)
    //     trajectory_->addPointEnd(Tpoint(path[i].x, path[i].y, 0));

    if (CONFIG.show_visualization)
        visualizeDCosts(dcosts_);
}

void Scene::visualizeDCosts(std::map<int, double> dcosts, bool skeleton) 
{
    for(int i = 0; i < dcosts.size(); i++) 
    {
        // get cell coordinates
        int cells_per_row = (int)floor(getGrid()->width / CONFIG.width);
        std::div_t res = std::div(i, cells_per_row);
        double cx = res.rem * CONFIG.width;
        double cy = res.quot * CONFIG.height;

        // draw
        QwtLinearColorMap m_colorMap;
        generateColormMap(m_colorMap);
        QwtInterval interval(0.0, 1.0);

        double cost = dcosts[i];
        double scaled_cost = (cost - CONFIG.dmin) / ((CONFIG.dmax - CONFIG.dmin));
        QColor col = m_colorMap.color(interval, scaled_cost);

        if (skeleton)
            guiScene->addRect(cx, cy, CONFIG.width, CONFIG.height, QPen(col));
        else
            guiScene->addRect(cx, cy, CONFIG.width, CONFIG.height, QPen(col), QBrush(col));
    }
}

void Scene::visualizePath2()
{
    //    QPen pen(randomColor(random_int(0,9)), 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen pen(Qt::white, 0.3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    std::vector<Tpoint> path = trajectory_->getPath();

    double xl = path[0].x, yl = path[0].y;
    for (size_t i = 1; i < path.size() - 1; i++) {
        guiScene->addLine(xl, yl, path[i].x, path[i].y, pen);
        xl = path[i].x;
        yl = path[i].y;
    }
}


/// =========================================================================
/// Simple helpers
/// =========================================================================

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





/// =========================================================================
/// Metrics
/// =========================================================================
void Scene::logExperimentData()
{
    FeatureICRA f;
    f.fill(0);

    FeatureICRA fc;


    if (CONFIG.feature_set == 1.0)
        fc = ficra_->computeCellFeaturesFS1(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);
    else if (CONFIG.feature_set == 2.0)
        fc = ficra_->computeCellFeaturesFS2(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);
    else if (CONFIG.feature_set == 3.0)
        fc = ficra_->computeCellFeaturesFS3(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);
    else if (CONFIG.feature_set == 4.0)
        fc = ficra_->computeCellFeaturesFS4(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);
    else if (CONFIG.feature_set == 5.0)
        fc = ficra_->computeCellFeaturesFS5(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);
    else if (CONFIG.feature_set == 6.0)
        fc = ficra_->computeCellFeaturesFS6(all_agents_, robot_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);

#ifdef FLIP
    if (CONFIG.feature_set == 6.0) {
        f = helbingFlip(fc);
    } else {
        f = flipFeature(fc, icra_policy_);
    }
#endif



    std::cout << robot_->getx() << " " << robot_->gety() << " " << timestep << " ";
    std::cout << anisotropics_.towards << " ";
    std::cout << anisotropics_.orthogonal << " ";
    std::cout << anisotropics_.behind << " ";
    std::cout << proxemics_.intimate << " ";
    std::cout << proxemics_.personal << " ";
    std::cout << proxemics_.social << " ";
    std::cout << proxemics_.publik << " ";
    std::cout << closest_distance_ << " ";
    std::cout << f[0]<<" "<<f[1]<<" "<<f[2]<<" "<<f[3]<<" "<<f[4]<<" "<<f[5]<<" "<<f[6]<<" "<<f[7]<<" "<<f[8]<<" ";
    std::cout <<f[9]<<" "<<f[10]<<" "<<f[11]<< " " <<f[12] << " " << ficra_->computeFeaturesCost(f, icra_policy_) <<endl;

    std::cout << std::endl;
}

void Scene::updateMetrics()
{
    //    double rmax = 3.2;
    double rmax = CONFIG.sensor_horizon;

    std::vector<Ped::Tagent*> potentials = getAgentsInRange(all_agents_, robot_->getx(), robot_->gety(), rmax);
    std::vector<Ped::Tagent*> neighbors = getAgentsInRange(all_agents_, robot_->getx(), robot_->gety(), CONFIG.sensor_horizon);

    for (vector<Ped::Tagent*>::const_iterator iter = potentials.begin(); iter != potentials.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        // get the id of the agent that the robot is intruding onto
        int intrusion = robotIntruding(a, robot_, neighbors.size());

        if ( (intrusion >= 0))
        {
            // decide direction of intrusion
            switch(getIntrusionDirection(a, robot_)) {
            case TOWARDS:
                anisotropics_.towards++;
                break;
            case ORTHOGONAL:
                anisotropics_.orthogonal++;
                break;
            case AWAY:
                anisotropics_.behind++;
                break;
            case NODIR:
                break;
            }
        }

        /// update closest distance
        double dist = eDist(robot_->getx(), robot_->gety(), a->getx(), a->gety());
        if (dist < closest_distance_ )
            closest_distance_ = dist;
    }

    Tpoint rp(robot_->getx(), robot_->gety(),0);
    proxemics_.intimate += (countAgentsInZone(all_agents_, rp, 0, CONFIG.intimate, robot_->gettype()));
    proxemics_.personal += (countAgentsInZone(all_agents_, rp, CONFIG.intimate, CONFIG.personal, robot_->gettype()));
    proxemics_.social += (countAgentsInZone(all_agents_, rp, CONFIG.personal, CONFIG.social, robot_->gettype()));
    proxemics_.publik += (countAgentsInZone(all_agents_, rp, CONFIG.social, CONFIG.publik, robot_->gettype()));
}





void Scene::generatePath(std::vector<std::vector<GNode> >& paths, GNode st)
{
    // prep start and goal nodes
    Tpoint gc = getEnclosingCell(getGrid(), goal_.x, goal_.y);
    GNode gl(gc.x*CONFIG.width, gc.y*CONFIG.height);

    sgraph_ = new GenericSearchGraphDescriptor<GNode,double>();
    sgraph_->getHashBin_fp = &getHashBin;
    sgraph_->isAccessible_fp = &isAccessible;
    sgraph_->getSuccessors_fp = &getSuccessors;
    sgraph_->getHeuristics_fp = &getHeuristics;

    sgraph_->hashTableSize = getGrid()->width * getGrid()->height;
    sgraph_->hashBinSizeIncreaseStep = 1024;

    GNode start(st.x*CONFIG.width, st.y*CONFIG.height);
    sgraph_->SeedNode = start;
    sgraph_->TargetNode = gl;


    // plan and get the paths
    A_star_planner<GNode,double>  planner;
    planner.init( *sgraph_ );
    planner.plan();

    // std::vector<double> costs = planner.getPlannedPathCosts();
    // std::cout << "costs size " << costs[0] << endl;

    // retrieve the paths
    paths = planner.getPlannedPaths();
    //    std::cout << "Found [" << paths.size() << "] paths" << std::endl;
}

void Scene::convertPathToTrajectory(std::vector<GNode>& path, bool replan)
{
    trajectory_->reset();

    int i;
    if (replan)
        i = path.size()-2;
    else
        i = path.size()-1;

    for( ; i >= 0; i--)
    {
        GNode node = path[i];
        trajectory_->addPointEnd(Tpoint(node.x, node.y, 0));
        //        trajectory_->addPointEnd(Tpoint(node.x+cellwidth/2, node.y-cellheight/2, 0));     // for using cell centers
    }
}


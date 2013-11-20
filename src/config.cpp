// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

// Includes
#include "config.h"


// initialize static value
Config* Config::Config::instance = NULL;


Config::Config() {
    // make sure this reflects what is set as default in the suer interface!  --chgloor 2012-01-13
    guiShowWaypoints = false;
    simWallForce = 10;
    simPedForce = 10;
    // simPedForce = 2.1;
    simSpeed = 1000.0/30;
    mlLookAhead = true;
    showForces = false;
    showDirection = true;
    simh = 0.1;
    // simh = 0.5;

    fmax = 0.0;
    fmin = 10000.0;
    dmin = 10000.0;
    dmax = 0.0;
    obstacle_positions.clear();

    /// setup the feature thresholds
    densities = new double[3];
    densities[0] = 0.0; densities[1] = 2.0; densities[2] = 5.0;

    velocities = new double[3];
    velocities[0] = 0.0; velocities[1] = 0.015; velocities[2] = 0.025;

    angles = new double[3];
    angles[0] = -1.0; angles[1] = cos( 3 * M_PI / 4 ); angles[2] = cos( M_PI / 4 );
}

Config& Config::getInstance() {
    if(instance == NULL)
        instance = new Config();

    return *instance;
}

void Config::setGuiShowWaypoints(bool value) {
    guiShowWaypoints = value;
    emit waypointVisibilityChanged(value);
}

void Config::setSimWallForce(double value) {
    simWallForce = value;
}

void Config::setSimPedForce(double value) {
    simPedForce = value;
}

void Config::setSimSpeed(int value) {
    simSpeed = value;
}

void Config::readParameters(std::string filename)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    robot_speed = pt.get<double>("Robot.speed");
    robot_direction = pt.get<double>("Robot.direction");

    int dm = pt.get<int>("Robot.drive_mode");
    switch(dm) {
    case 0:
        drive_mode = FOLLOW_PATH;
        break;
    case 1:
        drive_mode = LOCAL_PLANNER;
        break;
    case 2:
        drive_mode = FOLLOW_POLICY;
        break;
    case 3:
        drive_mode = TELEOP;
        break;
    default:
        drive_mode = TELEOP;
        break;
    }

    step_size = pt.get<double>("Robot.step_size");
    touch_radius = pt.get<double>("Robot.reach_radius");

    double ut = pt.get<double>("CostMap.update_type");
    if (ut == 0.0) update_type = LOCAL;
    else update_type = ORACLE;

    double algo = pt.get<double>("CostMap.algorithm");
    if (algo == 1.0) run_algorithm = ASTAR;
    else run_algorithm = DIJKSTRA;

    double vis = pt.get<double>("CostMap.visualize");
    if (vis == 1.0) show_visualization = true;
    else show_visualization = false;

    sensor_horizon = pt.get<double>("CostMap.sensor_radius");

    intimate = pt.get<double>("Proxemics.intimate");
    personal = pt.get<double>("Proxemics.personal");
    social = pt.get<double>("Proxemics.social");
    publik = pt.get<double>("Proxemics.public");

    width = pt.get<double>("Grid.width") * (20);    // converting metres to grid units
    height = pt.get<double>("Grid.height") * (20);
    maxh = pt.get<double>("Grid.max_horizontal") * (20);
    maxv = pt.get<double>("Grid.max_vertical") * (20);


    double sn = pt.get<double>("Scenario.sceneid");
    if (sn == 1.0) scenario = HALLWAY;
    else scenario = AIRPORT;

    feature_set = pt.get<double>("FeatureSet.feature_set");

    pt.clear();
}

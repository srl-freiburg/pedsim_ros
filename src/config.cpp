// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
//      pedsim - A microscopic pedestrian simulation system.
//      Copyright (c) 2011 by Christian Gloor

// Includes
#include "config.h"


// initialize static value
Config* Config::Config::instance = NULL;


Config::Config() 
{
    // make sure this reflects what is set as default in the suer interface!  --chgloor 2012-01-13
    guiShowWaypoints = false;
    simWallForce = 10;
    simPedForce = 10;
    // simSpeed = 1000.0/30;
    simSpeed = 0.05;
    mlLookAhead = true;
    showForces = false;
    showDirection = true;
    simh = 0.1;

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

//void Config::setGuiShowWaypoints(bool value) {
//    guiShowWaypoints = value;
//}

//void Config::setSimWallForce(double value) {
//    simWallForce = value;
//}

//void Config::setSimPedForce(double value) {
//    simPedForce = value;
//}

//void Config::setSimSpeed(int value) {
//    simSpeed = value;
//}

void Config::readParameters(std::string filename)
{
   // TODO - more clean
}
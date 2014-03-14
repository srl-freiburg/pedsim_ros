// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


#include <pedsim_simulator/agent.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/waypoint.h>



Agent::Agent(double xIn, double yIn)
    : Ped::Tagent()
{
    // initialize Ped::Tagent
    Ped::Tagent::setType(Ped::Tagent::ADULT);
};

Agent::~Agent() {
}

/// Calculates the social force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::socialForce() const {
    Ped::Tvector t = Tagent::socialForce();
    return t;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::obstacleForce() const {
    Ped::Tvector t = Tagent::obstacleForce();
    return t;
}

/// Calculates the desired force. Same as in lib, but adds graphical representation
/// \author  chgloor
Ped::Tvector Agent::desiredForce() {
    Ped::Tvector t = Tagent::desiredForce();
    return t;
}

/// Calculates the look ahead force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::lookaheadForce(Ped::Tvector desired) const {
    Ped::Tvector t;
    if (CONFIG.mlLookAhead == true) {
        t = Tagent::lookaheadForce(desired);
    }
    return t;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
    Ped::Tvector t;

    return t;
}

/// move - calls the lib move and updates the graphics then
void Agent::move(double h) {
    //TODO: add these to rosparam
    Ped::Tagent::setfactorsocialforce(CONFIG.simPedForce);
    Ped::Tagent::setfactorobstacleforce(CONFIG.simWallForce);

    // use SFM as local controller for the robot
    if (Tagent::gettype() == Ped::Tagent::ROBOT )
    {
        Ped::Tagent::setfactorsocialforce(CONFIG.simPedForce);
        Ped::Tagent::setfactorobstacleforce(350);
        Ped::Tagent::setfactordesiredforce(1.5);
    }

    Ped::Tagent::move(h);
}

void Agent::addWaypoint(Waypoint* waypointIn) {
    // keep track of waypoints
    waypoints.append(waypointIn);

    // call the original method
    Ped::Tagent::addWaypoint(waypointIn);
}

void Agent::setPosition(double px, double py) {
    // call super class' method
    Ped::Tagent::setPosition(px, py, 0);
}


void Agent::setX(double xIn) {
    setPosition(xIn, gety());
}

void Agent::setY(double yIn) {
    setPosition(getx(), yIn);
}

void Agent::setType(Ped::Tagent::AgentType t) {
    // call super class' method
    Ped::Tagent::setType(t);
}

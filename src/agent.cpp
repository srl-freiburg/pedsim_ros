// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include "agent.h"
// → SGDiCoP
#include "config.h"
#include "scene.h"
#include "waypoint.h"
// → Qt
#include <QPen>
#include <QGraphicsScene>
#include <QSettings>


Agent::Agent(double xIn, double yIn)
    : Ped::Tagent()
{
    // initialize Ped::Tagent
    Ped::Tagent::setType(0);
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
/// \date    2012-01-17
void Agent::move(double h) {
    //TODO: only change this when the config changes
    Ped::Tagent::setfactorsocialforce(CONFIG.simPedForce);
    Ped::Tagent::setfactorobstacleforce(CONFIG.simWallForce);

    Ped::Tagent::move(h);

    updateView();
}

void Agent::updateView() {
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

    // update graphical representation
    updateView();
}


void Agent::setX(double xIn) {
    setPosition(xIn, gety());
}

void Agent::setY(double yIn) {
    setPosition(getx(), yIn);
}

void Agent::setType(int t) {
    // call super class' method
    Ped::Tagent::setType(t);

    // update graphical representation
    updateView();
}

void Agent::updateLookOnSelection(bool selectedIn) {
   
}

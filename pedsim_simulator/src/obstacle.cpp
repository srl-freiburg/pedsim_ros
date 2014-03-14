// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include <pedsim_simulator/obstacle.h>


Obstacle::Obstacle(double pax, double pay, double pbx, double pby)
{
};

Obstacle::~Obstacle() {
    // clean up
}

/// moves the obstacle to a new position
/// \date    2012-01-07
void Obstacle::setPosition(double pax, double pay, double pbx, double pby) {
    Tobstacle::setPosition(pax, pay, pbx, pby);
};

void Obstacle::setX1(double xIn) {
    //HACK: there's no way to access ax etc. directly
    setPosition(xIn, getay(), getbx(), getby());
}

void Obstacle::setY1(double yIn) {
    //HACK: there's no way to access ay etc. directly
    setPosition(getax(), yIn, getbx(), getby());
}

void Obstacle::setX2(double xIn) {
    //HACK: there's no way to access bx etc. directly
    setPosition(getax(), getay(), xIn, getby());
}

void Obstacle::setY2(double yIn) {
    //HACK: there's no way to access by etc. directly
    setPosition(getax(), getay(), getbx(), yIn);
}


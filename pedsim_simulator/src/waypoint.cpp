// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include <pedsim_simulator/waypoint.h>
#include <pedsim_simulator/config.h>



/// Description: set intial values
Waypoint::Waypoint(const QString& idIn, double px, double py, double pr)
	: Twaypoint(px, py, pr), id(idIn) 
{
};

Waypoint::~Waypoint() {
	// clean up
}

void Waypoint::setPosition(double xIn, double yIn) {
	// update position
	Ped::Twaypoint::setx(xIn);
	Ped::Twaypoint::sety(yIn);
}

void Waypoint::setx(double xIn) {
	// update position
	Ped::Twaypoint::setx(xIn);
}

void Waypoint::sety(double yIn) {
	// update position
	Ped::Twaypoint::sety(yIn);
}

void Waypoint::setr(double rIn) {
	// update position
	Ped::Twaypoint::setr(rIn);
}

Ped::Tvector Waypoint::getForce(double myx, double myy, double fromx, double fromy, bool& reached) {	
	return Twaypoint::getForce(myx, myy, fromx, fromy, &reached);
}


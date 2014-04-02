// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include <pedsim_simulator/element/waypoint.h>



Waypoint::Waypoint(const QString& nameIn)
	: name(nameIn) {
}

Waypoint::Waypoint(const QString& nameIn, const Ped::Tvector& positionIn)
	: Ped::Twaypoint(positionIn),
		name(nameIn) {
}

Waypoint::~Waypoint() {
	// clean up
}

QString Waypoint::getName() const {
	return name;
}

void Waypoint::setPosition(double xIn, double yIn) {
	// update position
	Ped::Twaypoint::setx(xIn);
	Ped::Twaypoint::sety(yIn);

	// inform users
	emit positionChanged(getx(), gety());
}

void Waypoint::setPosition(const Ped::Tvector& posIn) {
	setPosition(posIn.x, posIn.y);
}

void Waypoint::setx(double xIn) {
	// update position
	Ped::Twaypoint::setx(xIn);

	// inform user
	emit positionChanged(getx(), gety());
}

void Waypoint::sety(double yIn) {
	// update position
	Ped::Twaypoint::sety(yIn);

	// inform user
	emit positionChanged(getx(), gety());
}

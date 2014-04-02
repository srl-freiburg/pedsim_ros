// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/element/areawaypoint.h>

// → Qt
#include <QSettings>


AreaWaypoint::AreaWaypoint(const QString& nameIn, const Ped::Tvector& positionIn, double rIn)
	: Waypoint(nameIn, positionIn) {
	// initialize values
	radius = rIn;

}

AreaWaypoint::AreaWaypoint(const QString& nameIn, double xIn, double yIn, double rIn)
	: Waypoint(nameIn, Ped::Tvector(xIn, yIn)) {
	// initialize values
	radius = rIn;
}

AreaWaypoint::~AreaWaypoint() {
}

QString AreaWaypoint::getName() const {
	return name;
}

bool AreaWaypoint::isWithinArea(const Ped::Tvector& posIn) {
	Ped::Tvector diff = getPosition() - posIn;
	double distance = diff.length();

	return (distance <= radius);
}

Ped::Tvector AreaWaypoint::closestPoint(const Ped::Tvector& posIn, bool* withinWaypoint) const {
	Ped::Tvector diff = position - posIn;

	if(diff.length() <= radius) {
		if(withinWaypoint != NULL)
			*withinWaypoint = true;

		return posIn;
	}
	else {
		if(withinWaypoint != NULL)
			*withinWaypoint = false;

		Ped::Tvector direction = diff.normalized();
		return position + radius*direction;
	}
}

double AreaWaypoint::getRadius() const {
	return radius;
}

void AreaWaypoint::setRadius(double rIn) {
	// update radius
	radius = rIn;

	// inform user
	emit radiusChanged(radius);
}

QPointF AreaWaypoint::getVisiblePosition() const {
	return QPointF(getx(), gety());
}

void AreaWaypoint::setVisiblePosition(const QPointF& positionIn) {
	setPosition(positionIn.x(), positionIn.y());
}

QString AreaWaypoint::toString() const {
	return tr("AreaWaypoint '%1' (@%2,%3)").arg(name).arg(getx()).arg(gety());
}

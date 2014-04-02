// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/element/queueingwaypoint.h>

// → SGDiCoP
// #include "visual/areawaypointrepresentation.h"
// → PedSim
#include <libpedsim/ped_agent.h>
// → Qt
#include <QSettings>


QueueingWaypoint::QueueingWaypoint(const QString& nameIn, const Ped::Tvector& positionIn)
	: Waypoint(nameIn, positionIn) {
	// graphical representation
// 	representation = new QueueingWaypointRepresentation(this);
}

QueueingWaypoint::~QueueingWaypoint() {
	// clean up
// 	delete representation;
}

QString QueueingWaypoint::getName() const {
	return name;
}

Ped::Tvector QueueingWaypoint::getForce(const Ped::Tagent& agentIn, Ped::Tvector* desiredDirectionOut, bool* reached) const {
	if(reached != nullptr)
		*reached = false;

	// compute the force
	Ped::Tvector diff = position - agentIn.getPosition();
	double distance = diff.length();

	const double distanceThreshold = 1.0;
	if(distance >= distanceThreshold) {
		// trivial case: agent is far away
		Ped::Tvector desiredDirection = diff.normalized();
		Ped::Tvector force = (desiredDirection * agentIn.getVmax() - agentIn.getVelocity()) / agentIn.getRelaxationTime();
		if(desiredDirectionOut != nullptr)
			*desiredDirectionOut = desiredDirection;
		return force;
	}
	else {
		// agent is already very close to the waypoint
		Ped::Tvector velocity = agentIn.getVelocity();
		// → decelerate agent
		Ped::Tvector decelerationForce = -velocity / agentIn.getRelaxationTime();

		// → move agent to the correct place
		Ped::Tvector projection = velocity * agentIn.getRelaxationTime();
		Ped::Tvector projectedDiff = diff - projection;
		Ped::Tvector projectionForce = projectedDiff / agentIn.getRelaxationTime();

		Ped::Tvector force = decelerationForce + projectionForce;
		if(desiredDirectionOut != nullptr)
			*desiredDirectionOut = velocity;
		return force;
	}
}

Ped::Tvector QueueingWaypoint::closestPoint(const Ped::Tvector& posIn, bool* withinWaypoint) const {
	return position;
}

QPointF QueueingWaypoint::getVisiblePosition() const {
	return QPointF(getx(), gety());
}

void QueueingWaypoint::setVisiblePosition(const QPointF& positionIn) {
	setPosition(positionIn.x(), positionIn.y());
}

QString QueueingWaypoint::toString() const {
	return tr("QueueingWaypoint '%1' (@%2,%3)").arg(name).arg(getx()).arg(gety());
}

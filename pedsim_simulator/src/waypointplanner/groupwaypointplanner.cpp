// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "groupwaypointplanner.h"
// → SGDiCoP
#include "logging.h"
#include "element/agent.h"
#include "element/agentgroup.h"
#include "element/areawaypoint.h"
#include "element/waitingqueue.h"


GroupWaypointPlanner::GroupWaypointPlanner() {
	// initialize values
	group = nullptr;
	destination = nullptr;
}

bool GroupWaypointPlanner::setGroup(AgentGroup* groupIn) {
	group = groupIn;

	return true;
}

Waypoint* GroupWaypointPlanner::getDestination() const {
	return destination;
}

void GroupWaypointPlanner::setDestination(Waypoint* waypointIn) {
	destination = waypointIn;
}

Waypoint* GroupWaypointPlanner::getCurrentWaypoint() {
	return destination;
}

bool GroupWaypointPlanner::hasCompletedDestination() const {
	if(destination == nullptr) {
		WARN_LOG("GroupWaypointPlanner: No destination set!");
		return true;
	}

	// check whether group has reached waypoint
	Ped::Tvector com = group->getCenterOfMass();
	AreaWaypoint* areaWaypoint = dynamic_cast<AreaWaypoint*>(destination);
	if(areaWaypoint != nullptr) {
		return areaWaypoint->isWithinArea(com);
	}
	else {
		ERROR_LOG("Unknown Waypoint type: %1", destination->toString());
		return true;
	}
}

QString GroupWaypointPlanner::name() const {
	return tr("GroupWaypointPlanner");
}

QString GroupWaypointPlanner::toString() const {
	return tr("%1 (%2)").arg(name()).arg((group==nullptr)?"null":group->toString());
}

// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include <pedsim_simulator/element/agent.h>

// → SGDiCoP
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/agentstatemachine.h>

#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/force/force.h>

#include <pedsim_simulator/waypointplanner/waypointplanner.h>
// → Qt
#include <QSettings>


Agent::Agent() {
	// initialize
	// → Ped::Tagent
	Ped::Tagent::setType(0);
	Ped::Tagent::setForceFactorObstacle(CONFIG.forceObstacle);
	forceSigmaObstacle = CONFIG.sigmaObstacle;
	Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial);
	// → waypoints
	currentDestination = nullptr;
	waypointplanner = nullptr;
	// → state machine
	stateMachine = new AgentStateMachine(this);
	// → group
	group = nullptr;


	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorObstacleChanged(double)), this, SLOT(onForceFactorObstacleChanged(double)));
	connect(&CONFIG, SIGNAL(forceSigmaObstacleChanged(double)), this, SLOT(onForceSigmaObstacleChanged(double)));
	connect(&CONFIG, SIGNAL(forceFactorSocialChanged(double)), this, SLOT(onForceFactorSocialChanged(double)));
}

Agent::~Agent() {
	// clean up
	// → remove forces
	foreach(Force* currentForce, forces) {
		delete currentForce;
	}
}

void Agent::onForceFactorObstacleChanged(double valueIn) {
	// update force factor
	Ped::Tagent::setForceFactorObstacle(CONFIG.forceObstacle);
}

void Agent::onForceSigmaObstacleChanged(double valueIn) {
	// update force sigma
	Ped::Tagent::forceSigmaObstacle = valueIn;
}

void Agent::onForceFactorSocialChanged(double valueIn) {
	// update force factor
	Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial);
}

/// Calculates the desired force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::desiredForce() {
	Ped::Tvector force;
	if(!disabledForces.contains("Desired"))
		force = Tagent::desiredForce();

	// inform users
	emit desiredForceChanged(force.x, force.y);

	return force;
}

/// Calculates the social force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::socialForce() const {
	Ped::Tvector force;
	if(!disabledForces.contains("Social"))
		force = Tagent::socialForce();

	// inform users
	emit socialForceChanged(force.x, force.y);

	return force;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::obstacleForce() const {
	Ped::Tvector force;
	if(!disabledForces.contains("Obstacle"))
		force = Tagent::obstacleForce();

	// inform users
	emit obstacleForceChanged(force.x, force.y);

	return force;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
	// run additional forces
	Ped::Tvector forceValue;
	foreach(Force* force, forces) {
		// skip disabled forces
		if(disabledForces.contains(force->getName())) {
			// update graphical representation
			emit additionalForceChanged(force->getName(), 0, 0);
			continue;
		}

		// add force to the total force
		Ped::Tvector currentForce = force->getForce(desired);
		// → sanity checks
		if(!currentForce.isValid()) {
// 			WARN_LOG("Invalid Force: %1", force->getName());
			currentForce = Ped::Tvector();
		}
		forceValue += currentForce;

		// update graphical representation
		emit additionalForceChanged(force->getName(), currentForce.x, currentForce.y);
	}

	// inform users
	emit myForceChanged(forceValue.x, forceValue.y);

	return forceValue;
}

Ped::Twaypoint* Agent::getCurrentDestination() const {
	return currentDestination;
}

Ped::Twaypoint* Agent::updateDestination() {
	// assign new destination
	if(!destinations.isEmpty()) {
		if(currentDestination != nullptr) {
			// cycle through destinations
			Waypoint* previousDestination = destinations.takeFirst();
			destinations.append(previousDestination);
		}
		currentDestination = destinations.first();
	}

	return currentDestination;
}

void Agent::updateState() {
	// check state
	stateMachine->doStateTransition();
}

void Agent::move(double h) {
	// move the agent
	Ped::Tagent::move(h);

	// inform users
	emit positionChanged(getx(), gety());
	emit velocityChanged(getvx(), getvy());
	emit accelerationChanged(getax(), getay());
}

const QList<Waypoint*>& Agent::getWaypoints() const {
	return destinations;
}

bool Agent::setWaypoints(const QList<Waypoint*>& waypointsIn) {
	destinations = waypointsIn;
	return true;
}

bool Agent::addWaypoint(Waypoint* waypointIn) {
	destinations.append(waypointIn);
	return true;
}

bool Agent::removeWaypoint(Waypoint* waypointIn) {
	int removeCount = destinations.removeAll(waypointIn);

	return (removeCount > 0);
}

bool Agent::needNewDestination() const {
	if(waypointplanner == nullptr)
		return (!destinations.isEmpty());
	else {
		// ask waypoint planner
		return waypointplanner->hasCompletedDestination();
	}
}

Ped::Twaypoint* Agent::getCurrentWaypoint() const {
	// Note: this is different from getCurrentDestination()…

	// sanity checks
	if(waypointplanner == nullptr)
		return nullptr;

	// ask waypoint planner
	return waypointplanner->getCurrentWaypoint();
}

bool Agent::isInGroup() const {
	return (group != nullptr);
}

AgentGroup* Agent::getGroup() const {
	return group;
}

void Agent::setGroup(AgentGroup* groupIn) {
	group = groupIn;
}

bool Agent::addForce(Force* forceIn) {
	forces.append(forceIn);

	// inform users
	emit forceAdded(forceIn->getName());

	// report success
	return true;
}

bool Agent::removeForce(Force* forceIn) {
	int removeCount = forces.removeAll(forceIn);

	// inform users
	emit forceRemoved(forceIn->getName());

	// report success if a Behavior has been removed
	return (removeCount >= 1);
}

AgentStateMachine* Agent::getStateMachine() const {
	return stateMachine;
}

WaypointPlanner* Agent::getWaypointPlanner() const {
	return waypointplanner;
}

void Agent::setWaypointPlanner(WaypointPlanner* plannerIn) {
	waypointplanner = plannerIn;
}

QList<const Agent*> Agent::getNeighbors() const {
	// upcast neighbors
	QList<const Agent*> output;
	for(const Ped::Tagent* neighbor : neighbors) {
		const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
		if(upNeighbor != nullptr)
			output.append(upNeighbor);
	}

	return output;
}

void Agent::disableForce(const QString& forceNameIn) {
	// disable force by adding it to the list of disabled forces
	disabledForces.append(forceNameIn);
}

void Agent::enableAllForces() {
	// remove all forces from disabled list
	disabledForces.clear();
}

void Agent::setPosition(double xIn, double yIn) {
	// call super class' method
	Ped::Tagent::setPosition(xIn, yIn);

	// inform users
	emit positionChanged(xIn, yIn);
}

void Agent::setX(double xIn) {
	setPosition(xIn, gety());
}

void Agent::setY(double yIn) {
	setPosition(getx(), yIn);
}

void Agent::setType(int typeIn) {
	// call super class' method
	Ped::Tagent::setType(typeIn);

	// inform users
	emit typeChanged(typeIn);
}

Ped::Tvector Agent::getDesiredDirection() const {
	return desiredforce;
}

Ped::Tvector Agent::getWalkingDirection() const {
	return v;
}

Ped::Tvector Agent::getSocialForce() const {
	return socialforce;
}

Ped::Tvector Agent::getObstacleForce() const {
	return obstacleforce;
}

Ped::Tvector Agent::getMyForce() const {
	return myforce;
}

QPointF Agent::getVisiblePosition() const {
	return QPointF(getx(), gety());
}

void Agent::setVisiblePosition(const QPointF& positionIn) {
	// check and apply new position
	if(positionIn != getVisiblePosition())
		setPosition(positionIn.x(), positionIn.y());
}

QString Agent::toString() const {
	return tr("Agent %1 (@%2,%3)").arg(getId()).arg(getx()).arg(gety());
}

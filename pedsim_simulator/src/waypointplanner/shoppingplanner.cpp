// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "shoppingplanner.h"
// → SGDiCoP
#include "rng.h"
#include "scene.h"
#include "element/agent.h"
#include "element/attractionarea.h"
#include "element/areawaypoint.h"
// → Qt
#include <QSettings>


ShoppingPlanner::ShoppingPlanner() {
	// initialize values
	agent = nullptr;
	currentWaypoint = nullptr;
	attraction = nullptr;
	timeReached = 0;
}

void ShoppingPlanner::loseAttraction() {
	// reset
	delete currentWaypoint;
	currentWaypoint = nullptr;
	attraction = nullptr;
	timeReached = 0;

	// inform users
	emit lostAttraction();
}

bool ShoppingPlanner::setAgent(Agent* agentIn) {
	agent = agentIn;
	return true;
}

AttractionArea* ShoppingPlanner::getAttraction() const {
	return attraction;
}

bool ShoppingPlanner::setAttraction(AttractionArea* attractionIn) {
	attraction = attractionIn;

	// reset waypoint
	delete currentWaypoint;
	currentWaypoint = nullptr;

	return true;
}

Waypoint* ShoppingPlanner::getCurrentWaypoint() {
	if(hasCompletedWaypoint())
		currentWaypoint = getNextWaypoint();
	
	return currentWaypoint;
}

bool ShoppingPlanner::hasCompletedWaypoint() {
	if(currentWaypoint == nullptr)
		return true;

	// check whether agent has reached the waypoint and has been there for a given time
	const double distanceThreshold = 1.0;
	const double waitTime = 5.0;
	double distance = (agent->getPosition() - currentWaypoint->getPosition()).length();
	if(distance <= distanceThreshold) {
		double sceneTime = SCENE.getTime();
		if(timeReached == 0)
			timeReached = sceneTime;
		else if(timeReached - sceneTime >= waitTime) {
			return true;
		}
	}

	return false;
}

bool ShoppingPlanner::hasCompletedDestination() const {
	// Note: The shopping planner is never done.
	//       Change Planner via StateMachine!
	return false;
}

Waypoint* ShoppingPlanner::getNextWaypoint() {
	bool hadWaypoint = (currentWaypoint != nullptr);
	Waypoint* oldWaypoint = currentWaypoint;

	// set new waypoint
	//TODO: create random attraction point in attraction
	QString name = createWaypointName();
	Ped::Tvector position;
	if(!hadWaypoint) {
		//TODO: closest point or random point?
		position = getRandomAttractionPosition();
	}
	else {
		position = oldWaypoint->getPosition();
		// → add random offset
		position += createRandomOffset();
	}

	// → ensure that the position is within the area
	QRectF area(QPointF(0, 0), attraction->getSize());
	area.moveCenter(QPointF(attraction->getPosition().x, attraction->getPosition().y));
	position.x = qBound(area.left(), position.x, area.right());
	position.y = qBound(area.top(), position.y, area.bottom());

	// → create new waypoint
	currentWaypoint = new AreaWaypoint(name, position, 0.5);

	// reset reached time
	timeReached = 0;

	// remove previous waypoint
	delete oldWaypoint;
	oldWaypoint = nullptr;

	return currentWaypoint;
}

QString ShoppingPlanner::createWaypointName() const {
	return QString("AttractionHelper_A%1_Q%2").arg(agent->getId()).arg(attraction->getName());
}

Ped::Tvector ShoppingPlanner::getRandomAttractionPosition() const {
	Ped::Tvector randomPosition = attraction->getPosition();

	// → add random part
	QSizeF size = attraction->getSize();
	std::uniform_real_distribution<double> xDistribution(-size.width()/2, size.width()/2);
	std::uniform_real_distribution<double> yDistribution(-size.height()/2, size.height()/2);
	randomPosition += Ped::Tvector(xDistribution(RNG()), yDistribution(RNG()));

	return randomPosition;
}

Ped::Tvector ShoppingPlanner::createRandomOffset() const {
	const double radiusStd = 4;
	std::normal_distribution<double> radiusDistribution(0, radiusStd);
	double radius = radiusDistribution(RNG());
	std::uniform_real_distribution<double> angleDistribution(0, 360);
	double angle = angleDistribution(RNG());
	Ped::Tvector randomOffset = Ped::Tvector::fromPolar(Ped::Tangle::fromDegree(angle), radius);
	return randomOffset;
}

QString ShoppingPlanner::name() const {
	return tr("ShoppingPlanner");
}

QString ShoppingPlanner::toString() const {
	return tr("ShoppingPlanner (Agent: %1, Attraction: %2)")
		.arg((agent!=nullptr)?"null":QString::number(agent->getId()))
		.arg((attraction!=nullptr)?"null":attraction->toString());
}

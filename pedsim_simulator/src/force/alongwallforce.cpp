// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/alongwallforce.h>
// → SGDiCoP
#include "config.h"
// #include "logging.h"
#include "scene.h"
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/obstacle.h>
// → Qt
#include <QSettings>


AlongWallForce::AlongWallForce(Agent* agentIn)
	: Force(agentIn) {
	// initialize values
	QSettings settings;
	speedThreshold = settings.value("AlongWallForce/SpeedThreshold", 0.2).toDouble();
	distanceThreshold = settings.value("AlongWallForce/DistanceThreshold", 0.6).toDouble();
	angleThresholdDegree = settings.value("AlongWallForce/AngleThreshold", 20).toDouble();
	setFactor(CONFIG.forceAlongWall);

	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorAlongWallChanged(double)),
		this, SLOT(onForceFactorChanged(double)));
}

void AlongWallForce::onForceFactorChanged(double valueIn) {
	setFactor(valueIn);
}

Ped::Tvector AlongWallForce::getForce(Ped::Tvector walkingDirection) {
	if(agent == nullptr) {
// 		ERROR_LOG("Cannot compute AlongWallForce for null agent!");
		return Ped::Tvector();
	}

	// check whether the agent is stuck
	// → doesn't move
	if(agent->getVelocity().length() > speedThreshold)
		return Ped::Tvector();

	// → walks against an obstacle
	Ped::Tvector force;
	Ped::Tvector agentPosition = agent->getPosition();
	const QList<Obstacle*>& obstacles = SCENE.getObstacles();
	// → find closest obstacle
	double minDistance = INFINITY;
	Ped::Tvector minDiff;
	Obstacle* minObstacle = nullptr;
	foreach(Obstacle* currentObstacle, obstacles) {
		Ped::Tvector closestPoint = currentObstacle->closestPoint(agentPosition);
		Ped::Tvector diff = closestPoint - agentPosition;
		double distance = diff.length();
		if(distance < minDistance) {
			minObstacle = currentObstacle;
			minDiff = diff;
			minDistance = distance;
		}
	}
	
	// check distance to closest obstacle
	if(minDistance > distanceThreshold)
		return Ped::Tvector();

	// check whether closest point is in walking direction
	const Ped::Tangle angleThreshold = Ped::Tangle::fromDegree(angleThresholdDegree);
	Ped::Tangle angle = walkingDirection.angleTo(minDiff);
	if(angle > angleThreshold)
		return Ped::Tvector();

// 	DEBUG_LOG("Found Agent %1 to be stuck!", agent->getId());

	// set force
	// → project to find walking direction
	Ped::Tvector obstacleDirection = minObstacle->getEndPoint() - minObstacle->getStartPoint();
	bool projectionPositive = (Ped::Tvector::dotProduct(walkingDirection, obstacleDirection) >= 0);
	
	Ped::Tvector forceDirection = (projectionPositive) ? obstacleDirection : -obstacleDirection;
	forceDirection.normalize();

	// scale force
	force = factor * forceDirection;
	return force;
}

QString AlongWallForce::toString() const {
	return tr("AlongWallForce (factor: %2)")
		.arg(factor);
}

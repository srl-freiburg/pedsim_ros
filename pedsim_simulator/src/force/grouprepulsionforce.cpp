// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/grouprepulsionforce.h>
// → SGDiCoP
#include "config.h"
// #include "logging.h"
#include <pedsim_simulator/element/agent.h>
// → Qt
#include <QSettings>


GroupRepulsionForce::GroupRepulsionForce(Agent* agentIn)
	: Force(agentIn) {
	// initialize values
	setFactor(CONFIG.forceGroupRepulsion);
	QSettings settings;
	overlapDistance = settings.value("GroupRepulsionForce/OverlapDistance", 0.5).toDouble();

	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorGroupRepulsionChanged(double)),
		this, SLOT(onForceFactorGroupRepulsionChanged(double)));
}

void GroupRepulsionForce::onForceFactorGroupRepulsionChanged(double valueIn) {
	setFactor(valueIn);
}

void GroupRepulsionForce::setGroup(AgentGroup* groupIn) {
	group = groupIn;
}

const AgentGroup& GroupRepulsionForce::getGroup() const {
	return *group;
}

Ped::Tvector GroupRepulsionForce::getForce(Ped::Tvector walkingDirection) {
	// sanity checks
	if(group->isEmpty()) {
// 		ERROR_LOG("Computing GroupRepulsionForce for empty group!");
		return Ped::Tvector();
	}

	// compute group repulsion force
	Ped::Tvector force;
	// → iterate over all group members
	foreach(Agent* currentAgent, group->getMembers()) {
		// → we don't need to take the our agent into account
		if(agent == currentAgent)
			continue;
		
		// → compute relative distance vector
		Ped::Tvector diff = agent->getPosition() - currentAgent->getPosition();
		double distance = diff.length();
		
		// → check whether other agent is overlapping
		if(distance < overlapDistance)
			force += diff;
	}

	// there is no factor for myForce, hence we have to do it
	force *= factor;

	return force;
}

QString GroupRepulsionForce::toString() const {
	return tr("GroupRepulsionForce (factor: %1)").arg(factor);
}

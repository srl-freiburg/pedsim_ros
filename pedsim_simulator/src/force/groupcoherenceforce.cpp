// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/groupcoherenceforce.h>
// → SGDiCoP
#include <pedsim_simulator/config.h>
// #include "logging.h"
#include <pedsim_simulator/element/agent.h>
// → Qt
#include <QSettings>


GroupCoherenceForce::GroupCoherenceForce(Agent* agentIn)
	: Force(agentIn) {
	// initialize values
	QSettings settings;
	setFactor(CONFIG.forceGroupCoherence);
	usePaperVersion = settings.value("UsePaperDefinitions", false).toBool();

	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorGroupCoherenceChanged(double)),
		this, SLOT(onForceFactorGroupCoherenceChanged(double)));
}

void GroupCoherenceForce::onForceFactorGroupCoherenceChanged(double valueIn) {
	setFactor(valueIn);
}

void GroupCoherenceForce::setGroup(AgentGroup* groupIn) {
	group = groupIn;
}

const AgentGroup& GroupCoherenceForce::getGroup() const {
	return *group;
}

Ped::Tvector GroupCoherenceForce::getForce(Ped::Tvector walkingDirection) {
	// sanity checks
	if(group->isEmpty()) {
// 		ERROR_LOG("Computing GroupCoherenceForce for empty group!");
		return Ped::Tvector();
	}

	// compute group coherence force
	// → compute relative position of center of mass
	Ped::Tvector com = group->getCenterOfMass();
	Ped::Tvector relativeCoM = com - agent->getPosition();
	// → distance to center of mass
	double distance = relativeCoM.length();
	// → approximate maximal distance, according to paper
	const double maxDistance = ((double) group->memberCount() - 1) / 2;

	// compute force
	Ped::Tvector force;
	// → switch between definitions
	if(usePaperVersion) {
		// force according to paper
		// → check whether maximal distance has been exceeded
		if(distance >= maxDistance) {
			// compute force
			force = relativeCoM.normalized();

			// there is no factor for myForce, hence we have to do it
			force *= factor;

			return force;
		}
		else {
			// there is no force
			return Ped::Tvector();
		}
	}
	else {
		// modified force
		force = relativeCoM;

		// there is no factor for myForce, hence we have to do it
		//HACK: use smooth transition
		//      this doesn't follow the Moussaid paper, but it creates less abrupt changes
		double softenedFactor = factor * (tanh(distance - maxDistance)+1) / 2;
		force *= softenedFactor;

		// 	DEBUG_LOG("softenedFactor = %0 = %1 * (tanh(%2 - %3)+1) / 2", softenedFactor, factor, distance, maxDistance);

		return force;
	}
}

QString GroupCoherenceForce::toString() const {
	return tr("GroupCoherenceForce (factor: %1)").arg(factor);
}

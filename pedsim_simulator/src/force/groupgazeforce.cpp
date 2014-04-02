// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/groupgazeforce.h>
// → SGDiCoP
#include "config.h"
// #include "logging.h"
#include <pedsim_simulator/element/agent.h>
// → Qt
#include <QSettings>


GroupGazeForce::GroupGazeForce(Agent* agentIn)
	: Force(agentIn) {
	// initialize values
	QSettings settings;
	setFactor(CONFIG.forceGroupGaze);
	usePaperVersion = settings.value("UsePaperDefinitions", false).toBool();

	// connect signals
	connect(&CONFIG, SIGNAL(forceFactorGroupGazeChanged(double)),
		this, SLOT(onForceFactorGroupGazeChanged(double)));
}

void GroupGazeForce::onForceFactorGroupGazeChanged(double valueIn) {
	setFactor(valueIn);
}

void GroupGazeForce::setGroup(AgentGroup* groupIn) {
	group = groupIn;
}

const AgentGroup& GroupGazeForce::getGroup() const {
	return *group;
}

Ped::Tvector GroupGazeForce::getForce(Ped::Tvector walkingDirection) {
	// sanity checks
	if(group->isEmpty()) {
// 		ERROR_LOG("Computing GroupGazeForce for empty group!");
		return Ped::Tvector();
	}

	// 1-agent groups don't need to compute this
	//TODO: maybe we shouldn't add this behavior to such groups
	int memberCount = group->memberCount();
	if(memberCount <= 1)
		return Ped::Tvector();

	// compute group force
	Ped::Tvector com = group->getCenterOfMass();
	// → use center of mass without the current agent
	com = (1/(double) (memberCount-1)) * (memberCount * com - agent->getPosition());

	// compute relative position of center of mass
	Ped::Tvector relativeCoM = com - agent->getPosition();

	// ensure that center of mass is in angle of vision (phi) in radians
	Ped::Tangle visionAngle = Ped::Tangle::fromDegree(90);
	// → angle between walking direction and center of mass
	//TODO: move this to a generic place
	double elementProduct = Ped::Tvector::dotProduct(walkingDirection, relativeCoM);
	// note: acos() returns the not directed angle in [0, pi]
	Ped::Tangle comAngle = Ped::Tangle::fromRadian(acos(elementProduct / (walkingDirection.length() * relativeCoM.length())));
	
	// compute force
	// → compute gazing direction
	if(comAngle > visionAngle) {
		Ped::Tvector force;
		
		// → switch between definitions
		if(usePaperVersion) {
			// agent has to rotate
			Ped::Tangle necessaryRotation = comAngle-visionAngle;
			
			force = -necessaryRotation.toRadian() * walkingDirection;
		}
		else {
			// agent has to rotate
			//HACK: this isn't the specification of the Moussaid et al. paper!
			//      but it decreases the amount of abrupt changes
			//      problem: it more or less hard codes that agents want to stay in line
			double walkingDirectionSquared = walkingDirection.lengthSquared();
			double walkingDirectionDistance = elementProduct / walkingDirectionSquared;
			force = walkingDirectionDistance * walkingDirection;
		}

		// there is no factor for myForce, hence we have to do it
		force *= factor;

		return force;
	}
	else {
		// no force
		return Ped::Tvector();
	}
}

QString GroupGazeForce::toString() const {
	return tr("GroupGazeForce (factor: %1)").arg(factor);
}

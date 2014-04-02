// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/force/force.h>

// → SGDiCoP
#include <pedsim_simulator/element/agent.h>


Force::Force(Agent* agentIn)
	: agent(agentIn), factor(1) {
}

void Force::setFactor(double factorIn) {
	factor = factorIn;
}

double Force::getFactor() const {
	return factor;
}

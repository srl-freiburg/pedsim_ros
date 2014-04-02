// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _grouprepulsionforce_h_
#define _grouprepulsionforce_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/force/force.h>
#include <pedsim_simulator/element/agentgroup.h>


class GroupRepulsionForce : public Force {
	Q_OBJECT

	// Constructor and Destructor
public:
	GroupRepulsionForce(Agent* agentIn);


	// Slots
public slots:
	void onForceFactorGroupRepulsionChanged(double valueIn);


	// Methods
public:
	void setGroup(AgentGroup* groupIn);
	const AgentGroup& getGroup() const;
	
	// → Force Implementations
public:
	virtual QString getName() const { return "GroupRepulsion"; };
	virtual Ped::Tvector getForce(Ped::Tvector walkingDirection);
	virtual QString toString() const;


	// Attributes
protected:
	AgentGroup* group;
	double overlapDistance;
};

#endif

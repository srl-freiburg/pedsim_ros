// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _alongwallforce_h_
#define _alongwallforce_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/force/force.h>


class AlongWallForce : public Force {
	Q_OBJECT

	// Constructor and Destructor
public:
	AlongWallForce(Agent* agentIn);


	// Slots
public slots:
	void onForceFactorChanged(double valueIn);


	// Methods
	// → Force Implementations
public:
	virtual QString getName() const { return "AlongWall"; };
	virtual Ped::Tvector getForce(Ped::Tvector walkingDirection);
	virtual QString toString() const;


	// Attributes
protected:
	double speedThreshold;
	double angleThresholdDegree;
	double distanceThreshold;
};

#endif

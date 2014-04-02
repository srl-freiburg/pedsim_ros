// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _randomforce_h_
#define _randomforce_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/force/force.h>


class RandomForce : public Force {
	Q_OBJECT

	// Constructor and Destructor
public:
	RandomForce(Agent* agentIn);


	// Slots
public slots:
	void onForceFactorChanged(double valueIn);


	// Methods
public:
	void setFadingTime(double durationIn);
	double getFadingTime() const;
	
protected:
	static Ped::Tvector computeNewDeviation();
	
	// → Force Implementations
public:
	virtual QString getName() const { return "Random"; };
	virtual Ped::Tvector getForce(Ped::Tvector walkingDirection);
	virtual QString toString() const;


	// Attributes
protected:
	double fadingDuration;
	Ped::Tvector lastDeviation;
	Ped::Tvector nextDeviation;
};

#endif

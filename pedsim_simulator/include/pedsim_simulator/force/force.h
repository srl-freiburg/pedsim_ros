// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _force_h_
#define _force_h_

// Includes
// → PedSim
#include <libpedsim/ped_vector.h>
// → Qt
#include <QObject>

// Forward Declarations
class Agent;


class Force : public QObject {
	Q_OBJECT

	// Constructor and Destructor
public:
	Force(Agent* agentIn);


	// Methods
	void setFactor(double factorIn);
	double getFactor() const;
public:
	virtual QString getName() const = 0;
	virtual Ped::Tvector getForce(Ped::Tvector walkingDirection) = 0;
	virtual QString toString() const = 0;


	// Attributes
protected:
	Agent* const agent;
	double factor;
};

#endif

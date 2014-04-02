// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _shoppingplanner_h_
#define _shoppingplanner_h_

// Includes
// → SGDiCoP
#include "waypointplanner.h"
// → PedSim
#include "ped_vector.h"


// Forward Declarations
class Agent;
class AttractionArea;


class ShoppingPlanner : public WaypointPlanner {
	Q_OBJECT

	// Constructor and Destructor
public:
	ShoppingPlanner();


	// Signals
signals:
	void lostAttraction();


public slots:
	void loseAttraction();


	// Methods
public:
	bool setAgent(Agent* agentIn);

	// → Waypoints
	AttractionArea* getAttraction() const;
	bool setAttraction(AttractionArea* attractionIn);

protected:
	// → Helper Methods
	QString createWaypointName() const;
	Ped::Tvector getRandomAttractionPosition() const;
	Ped::Tvector createRandomOffset() const;

	// → WaypointPlanner Overrides
public:
	static Type getPlannerType() { return WaypointPlanner::Individual; };
	virtual Waypoint* getCurrentWaypoint();
	virtual Waypoint* getNextWaypoint();
	virtual bool hasCompletedWaypoint();
	virtual bool hasCompletedDestination() const;

	virtual QString name() const;
	virtual QString toString() const;


	// Attributes
protected:
	Agent* agent;
	AttractionArea* attraction;
	// → Waypoints
	Waypoint* currentWaypoint;
	double timeReached;
};

#endif

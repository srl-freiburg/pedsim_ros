// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _individualwaypointplanner_h_
#define _individualwaypointplanner_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/waypointplanner/waypointplanner.h>

// Forward Declarations
class AgentGroup;


class IndividualWaypointPlanner : public WaypointPlanner {
	Q_OBJECT

	// Constructor and Destructor
public:
	IndividualWaypointPlanner();


	// Methods
public:
	bool setAgent(Agent* agentIn);

	// → Waypoints
	Waypoint* getDestination() const;
	void setDestination(Waypoint* waypointIn);

	// → WaypointPlanner Overrides
public:
	static Type getPlannerType() { return WaypointPlanner::Individual; };
	virtual Waypoint* getCurrentWaypoint();
	virtual bool hasCompletedDestination() const;
	
	virtual QString name() const;
	virtual QString toString() const;


	// Attributes
protected:
	Agent* agent;

	// → Waypoints
	Waypoint* destination;
};

#endif

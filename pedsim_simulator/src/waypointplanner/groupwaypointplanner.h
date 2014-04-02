// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _groupwaypointplanner_h_
#define _groupwaypointplanner_h_

// Includes
// → SGDiCoP
#include "waypointplanner.h"

// Forward Declarations
class AgentGroup;


class GroupWaypointPlanner : public WaypointPlanner {
	Q_OBJECT

	// Constructor and Destructor
public:
	GroupWaypointPlanner();


	// Methods
public:
	// → Waypoint
	Waypoint* getDestination() const;
	void setDestination(Waypoint* waypointIn);

	// → WaypointPlanner Overrides
public:
	static Type getPlannerType() { return WaypointPlanner::Group; };
	virtual Waypoint* getCurrentWaypoint();
	virtual bool hasCompletedDestination() const;

	virtual QString name() const;
	virtual QString toString() const;

	virtual bool setGroup(AgentGroup* groupIn);


	// Attributes
protected:
	AgentGroup* group;

	// → Waypoints
	Waypoint* destination;
};

#endif

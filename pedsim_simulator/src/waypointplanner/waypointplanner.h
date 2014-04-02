// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _waypointplanner_h_
#define _waypointplanner_h_

// Includes
// → Qt
#include <QObject>


// Forward Declarations
class Agent;
class AgentGroup;
class Waypoint;


class WaypointPlanner : public QObject {
	Q_OBJECT

	// Enums
public:
	typedef enum {
		Individual,
		Group,
		All
	} Type;


	// Constructor and Destructor
protected:
	WaypointPlanner();


	// Methods
public:
	static Type getPlannerType();
	virtual Waypoint* getCurrentWaypoint() = 0;
	virtual bool hasCompletedDestination() const = 0;
	
	virtual QString name() const = 0;
	virtual QString toString() const = 0;
};

#endif

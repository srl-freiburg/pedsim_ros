// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _queueingplanner_h_
#define _queueingplanner_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
// → PedSim
#include <libpedsim/ped_vector.h>


// Forward Declarations
class WaitingQueue;


class QueueingWaypointPlanner : public WaypointPlanner {
	Q_OBJECT

	typedef enum {
		Unknown,
		Approaching,
		Queued,
		MayPass
	} QueueingStatus;


	// Constructor and Destructor
public:
	QueueingWaypointPlanner();


	// Slots
protected slots:
	void onFollowedAgentPositionChanged(double xIn, double yIn);
	void onAgentMayPassQueue(int id);
	void onFollowedAgentLeftQueue();
	void onQueueEndPositionChanged(double xIn, double yIn);


	// Methods
public:
	void reset();

	// → Agent
	virtual Agent* getAgent() const;
	virtual bool setAgent(Agent* agentIn);

	// → WaitingQueue
	void setDestination(Waypoint* waypointIn);
	void setWaitingQueue(WaitingQueue* queueIn);
	WaitingQueue* getWaitingQueue() const;
	bool hasReachedQueueEnd() const;
	void activateApproachingMode();
	void activateQueueingMode();
	
protected:
	void addPrivateSpace(Ped::Tvector& queueEndIn) const;
	QString createWaypointName() const;

	// → WaypointPlanner Overrides
public:
	static Type getPlannerType() { return WaypointPlanner::Individual; };
	virtual Waypoint* getCurrentWaypoint();
	virtual Waypoint* getNextWaypoint();
	virtual bool hasCompletedWaypoint() const;
	virtual bool hasCompletedDestination() const;
	
	virtual QString name() const;
	virtual QString toString() const;


	// Attributes
protected:
	// → Agent
	Agent* agent;

	// → WaitingQueue
	WaitingQueue* waitingQueue;
	Waypoint* currentWaypoint;
	const Agent* followedAgent;
	QueueingStatus status;
};

#endif

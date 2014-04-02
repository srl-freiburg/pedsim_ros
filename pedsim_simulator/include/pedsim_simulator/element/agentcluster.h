// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _agentcluster_h_
#define _agentcluster_h_

// Includes
// → SGDiCoP
#include "agent.h"


// Forward Declarations
class Waypoint;
class WaitingQueue;
class AgentClusterRepresentation;


class AgentCluster : public ScenarioElement {
	Q_OBJECT


	// Constructor and Destructor
public:
	AgentCluster(double xIn = 0, double yIn = 0, int countIn = 1);
	virtual ~AgentCluster();


	// Signals
signals:
	void positionChanged(double x, double y);
	void typeChanged(int type);


	// Methods
public:
	QList<Agent*> dissolve();

	int getId() const;
	int getCount() const;
	void setCount(int countIn);
	const QList<Waypoint*>& getWaypoints() const;
	void addWaypoint(Waypoint* waypointIn);
	bool removeWaypoint(Waypoint* waypointIn);
	void addWaitingQueue(WaitingQueue* queueIn);
	bool removeWaitingQueue(WaitingQueue* queueIn);
	Ped::Tvector getPosition() const;
	void setPosition(const Ped::Tvector& positionIn);
	void setPosition(double px, double py);
	void setX(double xIn);
	void setY(double yIn);
	int getType() const;
	void setType(int typeIn);
	bool getShallCreateGroups() const;
	void setShallCreateGroups(bool shallCreateGroupsIn);
	QSizeF getDistribution() const;
	void setDistribution(double xIn, double yIn);
	void setDistributionWidth(double xIn);
	void setDistributionHeight(double yIn);

	// → ScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;


	// Attributes
protected:
	int id;
	Ped::Tvector position;
	int count;
	QSizeF distribution;
	int agentType;
	bool shallCreateGroups;
	QList<Waypoint*> waypoints;

	// → graphical representation
    AgentClusterRepresentation* representation;
};

#endif

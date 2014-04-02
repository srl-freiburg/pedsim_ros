// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _waitingqueue_h_
#define _waitingqueue_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/element/waypoint.h>

// → PedSim
#include <libpedsim/ped_vector.h>

// → Qt
#include <QPointF>


// Forward Declarations
class Agent;
// class WaitingQueueRepresentation;


class WaitingQueue : public Waypoint {
	Q_OBJECT

	// Constructor and Destructor
public:
	WaitingQueue(const QString& nameIn, Ped::Tvector positionIn, Ped::Tangle directionIn = Ped::Tangle::fromDegree(0));
    virtual ~WaitingQueue();


	// Signals
signals:
	void directionChanged(double radianAngle);
	// → Waiting Agents
	void agentMayPass(int id);
	void agentDequeued(int id);
	void queueLeaderChanged(int id);
	void queueEndChanged();
	void queueEndPositionChanged(double x, double y);


	// Slots
protected slots:
	void onTimeChanged(double timeIn);
	void onLastAgentPositionChanged(double xIn, double yIn);


	// Methods
public:
	Ped::Tangle getDirection() const;
	void setDirection(const Ped::Tangle& angleIn);
	void setDirection(double xIn, double yIn);
	void setDirection(const Ped::Tvector& directionIn);

	// → Queueing behavior
	bool isEmpty() const;
	Ped::Tvector getQueueEndPosition() const;
	const Agent* enqueueAgent(Agent* agentIn);
	bool dequeueAgent(Agent* agentIn);
	bool hasReachedWaitingPosition();
protected:
	void resetDequeueTime();
	void startDequeueTime();

protected:
	void informAboutEndPosition();

	// → Waypoint Overrides
public:
	virtual Ped::Tvector closestPoint(const Ped::Tvector& p, bool* withinWaypoint = NULL) const;

	// → ScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;


	// Attributes
protected:
	Ped::Tangle direction;

	QList<Agent*> queuedAgents;

	// → dequeueing
	double waitDurationMean;
	double waitDurationStd;
	double dequeueTime;
	
	// → graphical representation
// 	WaitingQueueRepresentation* representation;
};

#endif

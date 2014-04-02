// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _queueingwaypoint_h_
#define _queueingwaypoint_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/element/waypoint.h>
// → Qt
#include <QPointF>


class QueueingWaypoint : public Waypoint {
	Q_OBJECT

	// Constructor and Destructor
public:
	QueueingWaypoint(const QString& nameIn, const Ped::Tvector& positionIn);
	virtual ~QueueingWaypoint();


	// Signals
signals:
	void positionChanged(double x, double y);


	// Methods
public:
	QString getName() const;

	bool isWithinArea(const Ped::Tvector& posIn);

	// → Ped::Twaypoint Overrides
	virtual Ped::Tvector getForce(const Ped::Tagent& agentIn, Ped::Tvector* desiredDirectionOut = NULL, bool* reached = NULL) const;
	virtual Ped::Tvector closestPoint(const Ped::Tvector& posIn, bool* withinWaypoint = NULL) const;

	// → ScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;

};

#endif

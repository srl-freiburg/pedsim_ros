// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _areawaypoint_h_
#define _areawaypoint_h_

// Includes
// → SGDiCoP
#include "waypoint.h"
// → Qt
#include <QPointF>


// Forward Declarations
class AreaWaypointRepresentation;


class AreaWaypoint : public Waypoint {
	Q_OBJECT

	// Constructor and Destructor
public:
	AreaWaypoint(const QString& nameIn, const Ped::Tvector& positionIn, double rIn = 1);
	AreaWaypoint(const QString& nameIn, double xIn = 0, double yIn = 0, double rIn = 1);
	virtual ~AreaWaypoint();


	// Signals
signals:
	void radiusChanged(double radius);


	// Methods
public:
	QString getName() const;

	bool isWithinArea(const Ped::Tvector& posIn);

	virtual double getRadius() const;
	virtual void setRadius(double rIn);

	// → Ped::Twaypoint Overrides
	virtual Ped::Tvector closestPoint(const Ped::Tvector& posIn, bool* withinWaypoint = NULL) const;

	// → ScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;


	// Attributes
public:
// 	AreaWaypointRepresentation* representation;
	double radius;
};

#endif

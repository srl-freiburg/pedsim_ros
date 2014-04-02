// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _waypoint_h_
#define _waypoint_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/element/scenarioelement.h>
// → PedSim
#include <libpedsim/ped_waypoint.h>


class Waypoint : public ScenarioElement, public Ped::Twaypoint {
	Q_OBJECT

	// Constructor and Destructor
public:
	Waypoint(const QString& nameIn);
	Waypoint(const QString& nameIn, const Ped::Tvector& positionIn);
	virtual ~Waypoint();


	// Signals
signals:
	void positionChanged(double x, double y);


	// Methods
public:
	QString getName() const;
	// → Ped::Twaypoint Overrides
	virtual void setPosition(double xIn, double yIn);
	virtual void setPosition(const Ped::Tvector& posIn);
	virtual void setx(double xIn);
	virtual void sety(double yIn);


	// Attributes
protected:
	const QString name;
};

#endif

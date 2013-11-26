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
// → PedSim
#include "ped_waypoint.h"
#include <QtCore>

// Forward Declarations
class Scene;

class Waypoint : public Ped::Twaypoint
{
	// Constructor and Destructor
public:
    Waypoint(const QString& idIn, double x = 0, double y = 0, double r = 1);
    virtual ~Waypoint();


	// Methods
public:
	void setPosition(double xIn, double yIn);

	// → Ped::Twaypoint Overrides
	void setx(double xIn);
	void sety(double yIn);
	void setr(double rIn);

	/// returns the force into the direction of the waypoint
	Ped::Tvector getForce(double myx, double myy, double fromx, double fromy, bool& reached);

	// → ScenarioElement Overrides/Overloads
public:
	// → QGraphicsItem Overrides
	virtual int type() const { return Ped::Twaypoint::gettype(); }

	// Attributes
public:
	const QString id;

};

#endif

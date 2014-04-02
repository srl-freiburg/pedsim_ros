// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _obstacle_h_
#define _obstacle_h_

// Includes
// → SGDiCoP
#include <pedsim_simulator/element/scenarioelement.h>
// #include "visual/obstaclerepresentation.h"
// → PedSim
#include <libpedsim/ped_obstacle.h>
// → Qt
#include <QGraphicsLineItem>


class Obstacle : public ScenarioElement, public Ped::Tobstacle {
	Q_OBJECT

	// Constructor and Destructor
public:
	Obstacle(double ax = 0, double ay = 0, double bx = 1, double by = 1);
	virtual ~Obstacle();


	// Signals
signals:
	void positionChanged();


	// Methods
public:
	void setPosition(double ax, double ay, double bx, double by);
	void setPosition(const QPointF& startIn, const QPointF& endIn);
	void setX1(double xIn);
	void setY1(double yIn);
	void setX2(double xIn);
	void setY2(double yIn);

	// → ScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;


};

#endif

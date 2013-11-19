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
#include "scenarioelement.h"
// → PedSim
#include "ped_obstacle.h"
// → Qt
#include <QGraphicsLineItem>
#include <QWeakPointer>


// Forward Declarations
class Scene;


class Obstacle : public Ped::Tobstacle, public ScenarioElement, public QGraphicsLineItem {
	// Define Type
	// Needed by QGraphicsItem::type()
public:
	enum { Type = UserType + 2 };


	// Constructor and Destructor
public:
	Obstacle(const QWeakPointer<Scene>& sceneIn, double ax = 0, double ay = 0, double bx = 1, double by = 1);
	virtual ~Obstacle();


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
	void updateLookOnSelection(bool selectedIn);
	QString toString() const;

	// → QGraphicsItem Overrides
	virtual int type() const { return Type; }


	// Attributes
protected:
	QWeakPointer<Scene> scene;
};

#endif

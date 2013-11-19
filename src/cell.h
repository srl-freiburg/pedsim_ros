// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _cell_h_
#define _cell_h_

// Includes
// → Qt
#include <QList>
#include <QVariant>

// Forward Declarations
class QGraphicsScene;
class QGraphicsRectItem;


class Cell {
	// Constructor and Destructor
public:
	//TODO: use QRectF?
	Cell(double x, double y, double w, double h, QGraphicsScene* l);
	virtual ~Cell();


	inline double x() { return x_; }
	inline double y() { return y_; }


	// Attributes
public:
	QGraphicsScene* graphicsscene;
	QGraphicsRectItem* rect;

	QList<QVariant> values;

    bool hasAgent;
    bool occupied;

    double x_, y_;
};

#endif

// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include <pedsim_simulator/element/obstacle.h>

// → SGDiCoP
// #include "visual/obstaclerepresentation.h"
// → Qt
#include <QSettings>


Obstacle::Obstacle(double pax, double pay, double pbx, double pby)
	: Tobstacle(pax, pay, pbx, pby) {
	// add graphical representation
// 	representation = new ObstacleRepresentation(this);
};

Obstacle::~Obstacle() {
	// clean up
// 	delete representation;
}

/// moves the obstacle to a new position
void Obstacle::setPosition(double pax, double pay, double pbx, double pby) {
	Tobstacle::setPosition(pax, pay, pbx, pby);
	
	// inform users
	emit positionChanged();
}

void Obstacle::setPosition(const QPointF& startIn, const QPointF& endIn) {
	setPosition(startIn.x(), startIn.y(), endIn.x(), endIn.y());
}

void Obstacle::setX1(double xIn) {
	// update x1, keep the other values
	setPosition(xIn, getay(), getbx(), getby());
}

void Obstacle::setY1(double yIn) {
	// update y1, keep the other values
	setPosition(getax(), yIn, getbx(), getby());
}

void Obstacle::setX2(double xIn) {
	// update y2, keep the other values
	setPosition(getax(), getay(), xIn, getby());
}

void Obstacle::setY2(double yIn) {
	// update x2, keep the other values
	setPosition(getax(), getay(), getbx(), yIn);
}

QPointF Obstacle::getVisiblePosition() const {
	return QPointF(getax(), getay());
}

void Obstacle::setVisiblePosition(const QPointF& positionIn) {
	// compute new end position
	QPointF deltaPos(positionIn.x() - getax(), positionIn.y() - getay());
	QPointF endPos = QPointF(getbx(), getby()) + deltaPos;

	// set new position
	setPosition(positionIn.x(), positionIn.y(), endPos.x(), endPos.y());

	// inform users
	emit positionChanged();
}

QString Obstacle::toString() const {
	return tr("Obstacle (%1,%2 - %3,%4)")
		.arg(getax()).arg(getay())
		.arg(getbx()).arg(getby());
}

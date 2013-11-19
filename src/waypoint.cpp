// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include "waypoint.h"
// → SGDiCoP
#include "config.h"
#include "scene.h"
// → Qt
#include <QPen>
#include <QGraphicsScene>
#include <QSettings>


/// Description: set intial values
Waypoint::Waypoint(const QWeakPointer<Scene>& sceneIn, const QString& idIn, double px, double py, double pr)
	: Twaypoint(px, py, pr),
		ScenarioElement(sceneIn.data()), 
		QGraphicsEllipseItem(-pr, -pr, 2*pr, 2*pr),
		id(idIn) {
	scene = sceneIn;

	// graphical representation
	QSettings settings;
	QColor color = settings.value("Colors/Waypoint", QColor(66, 0, 0)).value<QColor>();
	QPen pen(color, 0.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	QBrush brush(color);
	setPen(pen);
	setBrush(brush);
	QGraphicsEllipseItem::setPos(px, py);
	setVisible(CONFIG.guiShowWaypoints);
	scene.data()->guiScene->addItem(this);

	// make waypoint selectable
	setFlags(QGraphicsItem::ItemIsSelectable);
};

Waypoint::~Waypoint() {
	// clean up
}

void Waypoint::setPosition(double xIn, double yIn) {
	// update position
	Ped::Twaypoint::setx(xIn);
	Ped::Twaypoint::sety(yIn);

	// update graphical representation
	QGraphicsEllipseItem::setPos(xIn, yIn);
	
	// inform user
	//TODO: emit positionChanged(x, y);
}

void Waypoint::setPosition(const QPointF& posIn) {
	setPosition(posIn.x(), posIn.y());
}

void Waypoint::setx(double xIn) {
	// update position
	Ped::Twaypoint::setx(xIn);

	// update graphical representation
	QGraphicsEllipseItem::setX(xIn);
	
	// inform user
	//TODO: emit positionChanged(x, y);
}

void Waypoint::sety(double yIn) {
	// update position
	Ped::Twaypoint::sety(yIn);

	// update graphical representation
	QGraphicsEllipseItem::setY(yIn);
	
	// inform user
	//TODO: emit positionChanged(x, y);
}

void Waypoint::setr(double rIn) {
	// update position
	Ped::Twaypoint::setr(rIn);

	// update graphical representation
	QGraphicsEllipseItem::setRect(-rIn, -rIn, 2*rIn, 2*rIn);

	// inform user
	//TODO: emit positionChanged(x, y);
}

Ped::Tvector Waypoint::getForce(double myx, double myy, double fromx, double fromy, bool& reached) {
	//TODO: this function isn't used
	
	//TODO: move this to a config changed slot
	QGraphicsEllipseItem::setVisible(CONFIG.guiShowWaypoints);
	
	return Twaypoint::getForce(myx, myy, fromx, fromy, &reached);
}

void Waypoint::updateLookOnSelection(bool selectedIn) {
	QBrush newBrush = brush();
	
	// use a dotted pattern, when selected, and a solid pattern otherwise.
	if(selectedIn)
		newBrush.setStyle(Qt::Dense2Pattern);
	else
		newBrush.setStyle(Qt::SolidPattern);
	
	setBrush(newBrush);
}

QString Waypoint::toString() const {
	return tr("Waypoint '%1' (@%2,%3)").arg(id).arg(getx()).arg(gety());
}

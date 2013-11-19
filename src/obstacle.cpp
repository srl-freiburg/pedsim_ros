// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include "obstacle.h"
// → SGDiCoP
#include "scene.h"
// → Qt
#include <QSettings>
#include <QPen>
#include <QGraphicsLineItem>
#include <QGraphicsScene>


Obstacle::Obstacle(const QWeakPointer<Scene>& sceneIn, double pax, double pay, double pbx, double pby)
    : Tobstacle(pax, pay, pbx, pby), QGraphicsLineItem(pax, pay, pbx, pby) {
    scene = sceneIn;

    // add graphical representation
    QSettings settings;
    QColor color = settings.value("Colors/Obstacle", Qt::blue).value<QColor>();
    QPen pen(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    setPen(pen);
    scene.data()->guiScene->addItem(this);

    // make obstacle selectable
    setFlags(QGraphicsItem::ItemIsSelectable);
};

Obstacle::~Obstacle() {
    // clean up
}

/// moves the obstacle to a new position
/// \date    2012-01-07
void Obstacle::setPosition(double pax, double pay, double pbx, double pby) {
    Tobstacle::setPosition(pax, pay, pbx, pby);
    setLine(pax, pay, pbx, pby);
};

void Obstacle::setPosition(const QPointF& startIn, const QPointF& endIn) {
    setPosition(startIn.x(), startIn.y(), endIn.x(), endIn.y());
}

void Obstacle::setX1(double xIn) {
    //HACK: there's no way to access ax etc. directly
    setPosition(xIn, getay(), getbx(), getby());
}

void Obstacle::setY1(double yIn) {
    //HACK: there's no way to access ay etc. directly
    setPosition(getax(), yIn, getbx(), getby());
}

void Obstacle::setX2(double xIn) {
    //HACK: there's no way to access bx etc. directly
    setPosition(getax(), getay(), xIn, getby());
}

void Obstacle::setY2(double yIn) {
    //HACK: there's no way to access by etc. directly
    setPosition(getax(), getay(), getbx(), yIn);
}

void Obstacle::updateLookOnSelection(bool selectedIn) {
    QPen newPen = pen();

    // use a dotted pattern, when selected, and a solid pattern otherwise.
    if(selectedIn)
        newPen.setStyle(Qt::DotLine);
    else
        newPen.setStyle(Qt::SolidLine);

    setPen(newPen);
}

QString Obstacle::toString() const {
    //HACK: libPedSim doesn't use const keyword for getax() etc.
    //      hence, we need to circumvent constness problems
    Obstacle* nonConstThis = const_cast<Obstacle*>(this);

    return tr("Obstacle (%1,%2 - %3,%4)")
            .arg(nonConstThis->getax()).arg(nonConstThis->getay())
            .arg(nonConstThis->getbx()).arg(nonConstThis->getby());
}

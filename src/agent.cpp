// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


// Includes
#include "agent.h"
// → SGDiCoP
#include "config.h"
#include "logging.h"
#include "scene.h"
#include "waypoint.h"
// → Qt
#include <QPen>
#include <QGraphicsScene>
#include <QSettings>


Agent::Agent(const QWeakPointer<Scene>& sceneIn, double xIn, double yIn)
    : QGraphicsRectItem(-0.5, -0.5, 1, 1) {
    scene = sceneIn;

    // initialize Ped::Tagent
    Ped::Tagent::setType(0);

    // graphical representation
    QSettings settings;
    QColor colorAgent = settings.value("Colors/Agent", Qt::white).value<QColor>();
    QPen penAgent(colorAgent, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QBrush brushAgent(colorAgent);
    QGraphicsScene* guiScene = scene.data()->guiScene;
    setPen(penAgent);
    setBrush(brushAgent);
    QGraphicsRectItem::setPos(xIn, yIn);
    guiScene->addItem(this);
    //TODO: setParentItem() already adds the element to the scene
    lineVelocity = new QGraphicsLineItem(this);
    lineVelocity->setPen(QPen(Qt::yellow, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    lineDesire = new QGraphicsLineItem(this);
    lineDesire->setPen(QPen(Qt::red, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    lineObstacle = new QGraphicsLineItem(this);
    lineObstacle->setPen(QPen(Qt::blue, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    lineSocial = new QGraphicsLineItem(this);
    lineSocial->setPen(QPen(Qt::green, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    lineLookahead = new QGraphicsLineItem(this);
    lineLookahead->setPen(QPen(Qt::magenta, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    lineGroup = new QGraphicsLineItem(this);
    lineGroup->setPen(QPen(Qt::cyan, 0.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    // make agent selectable
    setFlags(QGraphicsItem::ItemIsSelectable);
};

Agent::~Agent() {
    // clean up
    // → remove graphical representation
    delete lineVelocity;
    delete lineDesire;
    delete lineObstacle;
    delete lineSocial;
    delete lineLookahead;
    delete lineGroup;
}

/// Calculates the social force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::socialForce() const {
    Ped::Tvector t = Tagent::socialForce();
    if (CONFIG.showForces == true) {
        lineSocial->setLine(0, 0, CONFIG.simPedForce*t.x, CONFIG.simPedForce*t.y);
        lineSocial->setVisible(true);
    }
    else {
        lineSocial->setVisible(false);
    }
    return t;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::obstacleForce() const {
    Ped::Tvector t = Tagent::obstacleForce();
    if (CONFIG.showForces == true) {
        lineObstacle->setLine(0, 0, CONFIG.simWallForce*t.x, CONFIG.simWallForce*t.y);
        lineObstacle->setVisible(true);
    } else {
        lineObstacle->setVisible(false);
    }
    return t;
}

/// Calculates the desired force. Same as in lib, but adds graphical representation
/// \author  chgloor
Ped::Tvector Agent::desiredForce() {
    Ped::Tvector t = Tagent::desiredForce();
    if (CONFIG.showForces == true) {
        lineDesire->setLine(0, 0, t.x, t.y);
        lineDesire->setVisible(true);
    } else {
        lineDesire->setVisible(false);
    }
    return t;
}

/// Calculates the look ahead force. Same as in lib, but adds graphical representation
/// \date    2012-01-17
Ped::Tvector Agent::lookaheadForce(Ped::Tvector desired) const {
    Ped::Tvector t;
    if (CONFIG.mlLookAhead == true) {
        t = Tagent::lookaheadForce(desired);
    }
    if (CONFIG.showForces == true) {
        lineLookahead->setLine(0, 0, t.x, t.y);
        lineLookahead->setVisible(true);
    } else {
        lineLookahead->setVisible(false);
    }
    return t;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
    Ped::Tvector t;

    return t;
}

/// move - calls the lib move and updates the graphics then
/// \date    2012-01-17
void Agent::move(double h) {
    //TODO: only change this when the config changes
    Ped::Tagent::setfactorsocialforce(CONFIG.simPedForce);
    Ped::Tagent::setfactorobstacleforce(CONFIG.simWallForce);

    Ped::Tagent::move(h);

    updateView();
}

void Agent::updateView() {
    // change appearance according to agent type
    if(gettype() == 1) {
        setBrush(QBrush(Qt::red));
        setPen(QPen(Qt::red));
    }
    else if ( gettype() == 2) // type 2 == robot
    {
        setBrush( QBrush( Qt::green ) );
        setPen( QPen( Qt::green, 0.1 ) );
    }
    else if ( gettype() == 3)
    {
        setBrush( QBrush( Qt::yellow ) );
        setPen( QPen( Qt::yellow, 0.1 ) );
    }
    else if ( gettype() == 4)
    {
        setBrush( QBrush( Qt::cyan ) );
        setPen( QPen( Qt::cyan, 0.1 ) );
    }
    else {
        setBrush(QBrush(Qt::white));
        setPen(QPen(Qt::white));
    }


    // update graphical representation
    // → agent (position=upper left edge of rectangle)
    QGraphicsRectItem::setPos(getx(), gety());
    // → direction
    if(CONFIG.showDirection == true) {
        lineVelocity->setLine(0, 0, getvx(), getvy());
        lineVelocity->setVisible(true);
    }
    else {
        lineVelocity->setVisible(false);
    }
}

void Agent::addWaypoint(Waypoint* waypointIn) {
    // keep track of waypoints
    waypoints.append(waypointIn);

    // call the original method
    Ped::Tagent::addWaypoint(waypointIn);
}

void Agent::setPosition(double px, double py) {
    // call super class' method
    Ped::Tagent::setPosition(px, py, 0);

    // update graphical representation
    updateView();
}

void Agent::setPosition(const QPointF& posIn) {
    setPosition(posIn.x(), posIn.y());
}

void Agent::setX(double xIn) {
    setPosition(xIn, gety());
}

void Agent::setY(double yIn) {
    setPosition(getx(), yIn);
}

void Agent::setType(int t) {
    // call super class' method
    Ped::Tagent::setType(t);

    // update graphical representation
    updateView();
}

void Agent::updateLookOnSelection(bool selectedIn) {
    QBrush newBrush = brush();

    // use a dotted pattern, when selected, and a solid pattern otherwise.
    if(selectedIn)
        newBrush.setStyle(Qt::Dense2Pattern);
    else
        newBrush.setStyle(Qt::SolidPattern);

    setBrush(newBrush);
}
QString Agent::toString() const {
    return tr("Agent (@%1,%2)").arg(getx()).arg(gety());
}

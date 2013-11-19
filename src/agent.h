// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor


#ifndef _agent_h_
#define _agent_h_

// Includes
// → SGDiCoP
#include "scenarioelement.h"
// → PedSim
#include "ped_agent.h"
#include "ped_vector.h"
// → Qt
#include <QWeakPointer>
#include <QGraphicsRectItem>


// Forward Declarations
class Scene;
class Waypoint;


class Agent : public Ped::Tagent, public ScenarioElement, public QGraphicsRectItem {
    // Define Type
    // Needed by QGraphicsItem::type()
public:
    enum { Type = UserType + 1 };


    // Constructor and Destructor
public:
    Agent(const QWeakPointer<Scene>& sceneIn, double xIn = 0, double yIn = 0);
    virtual ~Agent();


    // Methods
public:
    void move(double h);
    Ped::Tvector socialForce() const;
    Ped::Tvector obstacleForce() const;
    Ped::Tvector desiredForce();
    Ped::Tvector lookaheadForce(Ped::Tvector desired) const;
    Ped::Tvector myForce(Ped::Tvector desired) const;

protected:
    void updateView();

    // → Ped::Tagent Overrides/Overloads
public:
    void addWaypoint(Waypoint* waypointIn);
    void setPosition(double px, double py);
    void setPosition(const QPointF& posIn);
    void setX(double xIn);
    void setY(double yIn);
    void setType(int t);

    // → ScenarioElement Overrides/Overloads
public:
    void updateLookOnSelection(bool selectedIn);
    QString toString() const;

    // → QGraphicsItem Overrides
    virtual int type() const { return Type; }


    // Attributes
public:
    // → reference to the scene
    QWeakPointer<Scene> scene;

    // → graphical representation
protected:
    //TODO: give them meaningful names
    QGraphicsLineItem* lineVelocity;
    QGraphicsLineItem* lineDesire;
    QGraphicsLineItem* lineObstacle;
    QGraphicsLineItem* lineSocial;
    QGraphicsLineItem* lineLookahead;
    QGraphicsLineItem* lineGroup;

    // → waypoints
    //HACK: we need to save them here again, because Ped::Tagent doesn't give
    //      access to them in any way
public:
    QList<Waypoint*> waypoints;
};

#endif

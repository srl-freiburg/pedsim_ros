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
#include <pedsim_simulator/element/scenarioelement.h>
// → PedSim
#include <libpedsim/ped_agent.h>
// → Qt
#include <QGraphicsRectItem>

// Forward Declarations
class AgentGroup;
// class AgentRepresentation;
class AgentStateMachine;
class Force;
class Waypoint;
class WaypointPlanner;


class Agent : public ScenarioElement, public Ped::Tagent {
	Q_OBJECT

	// Constructor and Destructor
public:
	Agent();
	virtual ~Agent();


	// Signals
signals:
	void positionChanged(double x, double y) const;
	void velocityChanged(double x, double y) const;
	void accelerationChanged(double x, double y) const;
	void desiredForceChanged(double x, double y) const;
	void obstacleForceChanged(double x, double y) const;
	void socialForceChanged(double x, double y) const;
	void myForceChanged(double x, double y) const;
	void additionalForceChanged(QString name, double x, double y) const;
	void reachedWaypoint() const;
	void typeChanged(int type);
	void forceAdded(QString name);
	void forceRemoved(QString name);


	// Slots
public slots:
	// → Forces
	void onForceFactorObstacleChanged(double valueIn);
	void onForceSigmaObstacleChanged(double valueIn);
	void onForceFactorSocialChanged(double valueIn);


	// Methods
public:
	// → waypoints
	const QList<Waypoint*>& getWaypoints() const;
	bool setWaypoints(const QList<Waypoint*>& waypointsIn);
	bool addWaypoint(Waypoint* waypointIn);
	bool removeWaypoint(Waypoint* waypointIn);

	Ped::Twaypoint* getCurrentDestination() const;
	bool needNewDestination() const;

	// → group
	bool isInGroup() const;
	AgentGroup* getGroup() const;
	void setGroup(AgentGroup* groupIn);

	// → forces
	bool addForce(Force* forceIn);
	bool removeForce(Force* forceIn);

	// → waypoint planner
	AgentStateMachine* getStateMachine() const;

	// → waypoint planner
	WaypointPlanner* getWaypointPlanner() const;
	void setWaypointPlanner(WaypointPlanner* plannerIn);

	// → direction, forces, neighbors
public:
	Ped::Tvector getDesiredDirection() const;
	Ped::Tvector getWalkingDirection() const;
	Ped::Tvector getSocialForce() const;
	Ped::Tvector getObstacleForce() const;
	Ped::Tvector getMyForce() const;
	QList<const Agent*> getNeighbors() const;
	void disableForce(const QString& forceNameIn);
	void enableAllForces();

	// → Ped::Tagent Overrides/Overloads
public:
	void updateState();
	void move(double h);
	Ped::Tvector desiredForce();
	Ped::Tvector socialForce() const;
	Ped::Tvector obstacleForce() const;
	Ped::Tvector myForce(Ped::Tvector desired) const;
	Ped::Twaypoint* getCurrentWaypoint() const;
	Ped::Twaypoint* updateDestination();
	void setPosition(double xIn, double yIn);
	void setX(double xIn);
	void setY(double yIn);
	void setType(int typeIn);

	// → VisibleScenarioElement Overrides/Overloads
public:
	virtual QPointF getVisiblePosition() const;
	virtual void setVisiblePosition(const QPointF& positionIn);
	QString toString() const;


	// Attributes
protected:
	// → graphical representation
// 	AgentRepresentation* representation;

	// → state machine
	AgentStateMachine* stateMachine;

	// → waypoints
	QList<Waypoint*> destinations;
	Waypoint* currentDestination;

	// → group
	AgentGroup* group;

	// → force
	QList<Force*> forces;
	QStringList disabledForces;
	
	// → waypoint planner
	WaypointPlanner* waypointplanner;
};

#endif

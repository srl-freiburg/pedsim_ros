/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#ifndef _agent_h_
#define _agent_h_

#include <pedsim/ped_agent.h>
#include <pedsim_simulator/element/scenarioelement.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <QGraphicsRectItem>  // TODO -remove qgraphics dependencies
#endif

// Forward Declarations
class AgentGroup;
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
  void setType(Ped::Tagent::AgentType typeIn);

  // → VisibleScenarioElement Overrides/Overloads
 public:
  virtual QPointF getVisiblePosition() const;
  virtual void setVisiblePosition(const QPointF& positionIn);
  QString toString() const;

  // Attributes
 protected:
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

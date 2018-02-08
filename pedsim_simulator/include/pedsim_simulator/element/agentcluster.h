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

#ifndef _agentcluster_h_
#define _agentcluster_h_

#include <pedsim_simulator/element/agent.h>

// Forward Declarations
class Waypoint;
class WaitingQueue;

class AgentCluster : public ScenarioElement {
  Q_OBJECT

  // Constructor and Destructor
 public:
  AgentCluster(double xIn = 0, double yIn = 0, int countIn = 1);
  virtual ~AgentCluster();

  // Signals
 signals:
  void positionChanged(double x, double y);
  void typeChanged(int type);

  // Methods
 public:
  QList<Agent*> dissolve();

  int getId() const;
  int getCount() const;
  void setCount(int countIn);
  const QList<Waypoint*>& getWaypoints() const;
  void addWaypoint(Waypoint* waypointIn);
  bool removeWaypoint(Waypoint* waypointIn);
  void addWaitingQueue(WaitingQueue* queueIn);
  bool removeWaitingQueue(WaitingQueue* queueIn);
  Ped::Tvector getPosition() const;
  void setPosition(const Ped::Tvector& positionIn);
  void setPosition(double px, double py);
  void setX(double xIn);
  void setY(double yIn);
  int getType() const;
  void setType(Ped::Tagent::AgentType typeIn);
  bool getShallCreateGroups() const;
  void setShallCreateGroups(bool shallCreateGroupsIn);
  QSizeF getDistribution() const;
  void setDistribution(double xIn, double yIn);
  void setDistributionWidth(double xIn);
  void setDistributionHeight(double yIn);

  // → ScenarioElement Overrides/Overloads
 public:
  virtual QPointF getVisiblePosition() const;
  virtual void setVisiblePosition(const QPointF& positionIn);
  QString toString() const;

  // Attributes
 protected:
  int id;
  Ped::Tvector position;
  int count;
  QSizeF distribution;
  Ped::Tagent::AgentType agentType;
  bool shallCreateGroups;
  QList<Waypoint*> waypoints;
};

#endif

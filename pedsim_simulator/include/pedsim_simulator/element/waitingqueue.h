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

#ifndef _waitingqueue_h_
#define _waitingqueue_h_

#include <pedsim/ped_vector.h>
#include <pedsim_simulator/element/waypoint.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <QPointF>
#endif

// Forward Declarations
class Agent;

class WaitingQueue : public Waypoint {
  Q_OBJECT

  // Constructor and Destructor
 public:
  WaitingQueue(const QString& nameIn, Ped::Tvector positionIn,
               Ped::Tangle directionIn = Ped::Tangle::fromDegree(0));
  virtual ~WaitingQueue();

  // Signals
 signals:
  void directionChanged(double radianAngle);
  // → Waiting Agents
  void agentMayPass(int id);
  void agentDequeued(int id);
  void queueLeaderChanged(int id);
  void queueEndChanged();
  void queueEndPositionChanged(double x, double y);

  // Slots
 protected slots:
  void onTimeChanged(double timeIn);
  void onLastAgentPositionChanged(double xIn, double yIn);

  // Methods
 public:
  Ped::Tangle getDirection() const;
  void setDirection(const Ped::Tangle& angleIn);
  void setDirection(double xIn, double yIn);
  void setDirection(const Ped::Tvector& directionIn);

  // → Queueing behavior
  bool isEmpty() const;
  Ped::Tvector getQueueEndPosition() const;
  const Agent* enqueueAgent(Agent* agentIn);
  bool dequeueAgent(Agent* agentIn);
  bool hasReachedWaitingPosition();

 protected:
  void resetDequeueTime();
  void startDequeueTime();

 protected:
  void informAboutEndPosition();

  // → Waypoint Overrides
 public:
  virtual Ped::Tvector closestPoint(const Ped::Tvector& p,
                                    bool* withinWaypoint = NULL) const;

  // → ScenarioElement Overrides/Overloads
 public:
  virtual QPointF getVisiblePosition() const;
  virtual void setVisiblePosition(const QPointF& positionIn);
  QString toString() const;

  // Attributes
 protected:
  Ped::Tangle direction;

  QList<Agent*> queuedAgents;

  // → dequeueing
  double waitDurationLambda;
  double dequeueTime;
};

#endif

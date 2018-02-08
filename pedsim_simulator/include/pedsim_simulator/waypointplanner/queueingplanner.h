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

#ifndef _queueingplanner_h_
#define _queueingplanner_h_

#include <pedsim/ped_vector.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>

// Forward Declarations
class WaitingQueue;

class QueueingWaypointPlanner : public WaypointPlanner {
  Q_OBJECT

  // TODO - change to enum class
  typedef enum { Unknown, Approaching, Queued, MayPass } QueueingStatus;

  // Constructor and Destructor
 public:
  QueueingWaypointPlanner();

  // Slots
 protected slots:
  void onFollowedAgentPositionChanged(double xIn, double yIn);
  void onAgentMayPassQueue(int id);
  void onFollowedAgentLeftQueue();
  void onQueueEndPositionChanged(double xIn, double yIn);

  // Methods
 public:
  void reset();

  // → Agent
  virtual Agent* getAgent() const;
  virtual bool setAgent(Agent* agentIn);

  // → WaitingQueue
  void setDestination(Waypoint* waypointIn);
  void setWaitingQueue(WaitingQueue* queueIn);
  WaitingQueue* getWaitingQueue() const;
  bool hasReachedQueueEnd() const;
  void activateApproachingMode();
  void activateQueueingMode();

 protected:
  void addPrivateSpace(Ped::Tvector& queueEndIn) const;
  QString createWaypointName() const;

  // → WaypointPlanner Overrides
 public:
  static Type getPlannerType() { return WaypointPlanner::Individual; };
  virtual Waypoint* getCurrentWaypoint();
  virtual Waypoint* getNextWaypoint();
  virtual bool hasCompletedWaypoint() const;
  virtual bool hasCompletedDestination() const;

  virtual QString name() const;

  // Attributes
 protected:
  // → Agent
  Agent* agent;

  // → WaitingQueue
  WaitingQueue* waitingQueue;
  Waypoint* currentWaypoint;
  const Agent* followedAgent;
  QueueingStatus status;
};

#endif

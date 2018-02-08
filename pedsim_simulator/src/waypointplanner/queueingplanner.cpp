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

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/queueingwaypoint.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/utilities.h>
#include <pedsim_simulator/waypointplanner/queueingplanner.h>

QueueingWaypointPlanner::QueueingWaypointPlanner() {
  // initialize values
  agent = nullptr;
  waitingQueue = nullptr;
  currentWaypoint = nullptr;
  followedAgent = nullptr;
  status = QueueingWaypointPlanner::Unknown;
}

void QueueingWaypointPlanner::onFollowedAgentPositionChanged(double xIn,
                                                             double yIn) {
  // sanity checks
  if (currentWaypoint == nullptr) {
    ROS_DEBUG(
        "Queued agent cannot update queueing position, because there's no "
        "waypoint set!");
    return;
  }

  Ped::Tvector followedPosition(xIn, yIn);
  addPrivateSpace(followedPosition);

  // HACK: don't update minor changes (prevent over-correcting)
  // TODO: integrate update importance to waypoint (force?)
  const double minUpdateDistance = 0.7;
  Ped::Tvector diff = followedPosition - currentWaypoint->getPosition();
  if (diff.length() < minUpdateDistance) return;

  currentWaypoint->setPosition(followedPosition);
}

void QueueingWaypointPlanner::onAgentMayPassQueue(int id) {
  // check who will leave queue
  if ((agent != nullptr) && (id == agent->getId())) {
    // the agent may pass
    // → update waypoint
    status = QueueingWaypointPlanner::MayPass;

    // remove references to old queue
    disconnect(waitingQueue, SIGNAL(agentMayPass(int)), this,
               SLOT(onAgentMayPassQueue(int)));
    disconnect(waitingQueue, SIGNAL(queueEndPositionChanged(double, double)),
               this, SLOT(onQueueEndPositionChanged(double, double)));
  } else if ((followedAgent != nullptr) && (id == followedAgent->getId())) {
    // followed agent leaves queue
    onFollowedAgentLeftQueue();
  }
}

void QueueingWaypointPlanner::onFollowedAgentLeftQueue() {
  // followed agent leaves queue
  // → remove all connections to old followed agent
  disconnect(followedAgent, SIGNAL(positionChanged(double, double)), this,
             SLOT(onFollowedAgentPositionChanged(double, double)));

  // → move to queue's front
  // HACK: actually we have to check our position and eventually bind to a new
  // followed agent
  Ped::Tvector queueingPosition = waitingQueue->getPosition();
  currentWaypoint->setPosition(queueingPosition);
}

void QueueingWaypointPlanner::onQueueEndPositionChanged(double xIn,
                                                        double yIn) {
  // there's nothing to do when the agent is already enqueued
  if (status != QueueingWaypointPlanner::Approaching) return;

  if (hasReachedQueueEnd()) {
    // change mode
    activateQueueingMode();
  } else {
    // don't update if the planner hasn't determined waypoint yet
    if (currentWaypoint == nullptr) return;

    // update destination
    Ped::Tvector newDestination(xIn, yIn);
    if (!waitingQueue->isEmpty()) addPrivateSpace(newDestination);
    currentWaypoint->setPosition(newDestination);
  }
}

void QueueingWaypointPlanner::reset() {
  // disconnect signals
  if (followedAgent != nullptr) {
    disconnect(followedAgent, SIGNAL(positionChanged(double, double)), this,
               SLOT(onFollowedAgentPositionChanged(double, double)));
  }
  if (waitingQueue != nullptr) {
    disconnect(waitingQueue, SIGNAL(agentMayPass(int)), this,
               SLOT(onAgentMayPassQueue(int)));
    disconnect(waitingQueue, SIGNAL(queueEndPositionChanged(double, double)),
               this, SLOT(onQueueEndPositionChanged(double, double)));
  }

  // unset variables
  status = QueueingWaypointPlanner::Unknown;
  delete currentWaypoint;
  currentWaypoint = nullptr;
  followedAgent = nullptr;
}

Agent* QueueingWaypointPlanner::getAgent() const { return agent; }

bool QueueingWaypointPlanner::setAgent(Agent* agentIn) {
  agent = agentIn;
  return true;
}

void QueueingWaypointPlanner::setDestination(Waypoint* waypointIn) {
  WaitingQueue* queue = dynamic_cast<WaitingQueue*>(waypointIn);

  // sanity checks
  if (queue == nullptr) {
    ROS_ERROR(
        "Waypoint provided to QueueingWaypointPlanner isn't a waiting queue! "
        "(%s)",
        (waypointIn == nullptr) ? "null"
                                : waypointIn->toString().toStdString().c_str());
    return;
  }

  // apply new destination
  setWaitingQueue(queue);
}

void QueueingWaypointPlanner::setWaitingQueue(WaitingQueue* queueIn) {
  // clean up old waiting queue
  reset();

  // set up for new waiting queue
  waitingQueue = queueIn;
  if (waitingQueue != nullptr) {
    status = QueueingWaypointPlanner::Approaching;
    // connect signals
    connect(waitingQueue, SIGNAL(agentMayPass(int)), this,
            SLOT(onAgentMayPassQueue(int)));
    connect(waitingQueue, SIGNAL(queueEndPositionChanged(double, double)), this,
            SLOT(onQueueEndPositionChanged(double, double)));
  }
}

WaitingQueue* QueueingWaypointPlanner::getWaitingQueue() const {
  return waitingQueue;
}

bool QueueingWaypointPlanner::hasReachedQueueEnd() const {
  const double endPositionRadius = 2.0;

  // sanity checks
  if (waitingQueue == nullptr) return false;

  Ped::Tvector queueEnd = waitingQueue->getQueueEndPosition();
  Ped::Tvector diff = queueEnd - agent->getPosition();

  if (diff.length() <= endPositionRadius)
    return true;
  else
    return false;
}

void QueueingWaypointPlanner::activateApproachingMode() {
  // update mode
  status = QueueingWaypointPlanner::Approaching;

  // set new waypoint
  QString waypointName = createWaypointName();
  Ped::Tvector destination = waitingQueue->getQueueEndPosition();

  // reset waypoint (remove old one)
  delete currentWaypoint;
  currentWaypoint = new QueueingWaypoint(waypointName, destination);

  // NOTE - wild experiment
  agent->disableForce("GroupCoherence");
  agent->disableForce("GroupGaze");
  agent->disableForce("GroupRepulsion");
}

void QueueingWaypointPlanner::activateQueueingMode() {
  // update mode
  status = QueueingWaypointPlanner::Queued;

  // set new waypoint
  QString waypointName = createWaypointName();
  Ped::Tvector queueingPosition;
  followedAgent = waitingQueue->enqueueAgent(agent);
  if (followedAgent != nullptr) {
    queueingPosition = followedAgent->getPosition();
    addPrivateSpace(queueingPosition);

    // keep updating the waypoint
    connect(followedAgent, SIGNAL(positionChanged(double, double)), this,
            SLOT(onFollowedAgentPositionChanged(double, double)));
  } else {
    queueingPosition = waitingQueue->getPosition();
  }

  // deactivate problematic forces
  agent->disableForce("Social");  /// Uncomment to enable chaotic queues mode
  agent->disableForce("Random");
  agent->disableForce("GroupCoherence");
  agent->disableForce("GroupGaze");
  agent->disableForce("GroupRepulsion");

  // reset waypoint (remove old one)
  delete currentWaypoint;
  currentWaypoint = new QueueingWaypoint(waypointName, queueingPosition);
}

/// Affects the behavior at the end of the queue and hence the shape
void QueueingWaypointPlanner::addPrivateSpace(Ped::Tvector& queueEndIn) const {
  std::uniform_real_distribution<double> spacing_range_(0.2, 0.8);
  std::uniform_real_distribution<double> heading_range_(-45.0, 45.0);

  // randomize spacing and heading in queues
  double privateSpaceDirection = heading_range_(RNG());
  Ped::Tangle orientation;
  orientation.setDegree(privateSpaceDirection);

  double privateSpaceDistance = spacing_range_(RNG());
  Ped::Tvector queueOffset(Ped::Tvector::fromPolar(
      waitingQueue->getDirection() + orientation, privateSpaceDistance));
  queueEndIn -= queueOffset;
}

QString QueueingWaypointPlanner::createWaypointName() const {
  return QString("QueueHelper_A%1_Q%2")
      .arg(agent->getId())
      .arg(waitingQueue->getName());
}

Waypoint* QueueingWaypointPlanner::getCurrentWaypoint() {
  if (hasCompletedWaypoint()) currentWaypoint = getNextWaypoint();

  return currentWaypoint;
}

Waypoint* QueueingWaypointPlanner::getNextWaypoint() {
  // sanity checks
  if (agent == nullptr) {
    ROS_DEBUG("Cannot determine queueing waypoint without agent!");
    return nullptr;
  }
  if (waitingQueue == nullptr) {
    ROS_DEBUG("Cannot determine queueing waypoint without waiting queues!");
    return nullptr;
  }

  // set mode
  if (hasReachedQueueEnd())
    activateQueueingMode();
  else
    activateApproachingMode();

  return currentWaypoint;
}

bool QueueingWaypointPlanner::hasCompletedWaypoint() const {
  if (currentWaypoint == nullptr) return true;

  // update waypoint, if necessary
  if (status == QueueingWaypointPlanner::Approaching) {
    if (hasReachedQueueEnd()) {
      return true;
    }
  }

  // check whether agent has reached waypoint
  return (status == QueueingWaypointPlanner::MayPass);
}

bool QueueingWaypointPlanner::hasCompletedDestination() const {
  if (waitingQueue == nullptr) {
    ROS_DEBUG("QueueingWaypointPlanner: No waiting queue set!");
    return true;
  }

  // check whether agent has reached waypoint
  return (status == QueueingWaypointPlanner::MayPass);
}

QString QueueingWaypointPlanner::name() const {
  return tr("QueueingWaypointPlanner");
}

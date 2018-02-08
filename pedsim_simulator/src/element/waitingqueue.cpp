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

#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>

WaitingQueue::WaitingQueue(const QString& nameIn, Ped::Tvector positionIn,
                           Ped::Tangle directionIn)
    : Waypoint(nameIn, positionIn), direction(directionIn) {
  // initialize values
  dequeueTime = INFINITY;
  waitDurationLambda = CONFIG.wait_time_beta;

  // connect signals
  connect(&SCENE, SIGNAL(sceneTimeChanged(double)), this,
          SLOT(onTimeChanged(double)));
}

WaitingQueue::~WaitingQueue() {}

void WaitingQueue::onTimeChanged(double timeIn) {
  // skip when there is none
  if (queuedAgents.empty()) {
    return;
  }

  Agent* firstInLine = queuedAgents.first();

  // check whether waiting started
  if (std::isinf(dequeueTime)) {
    if (hasReachedWaitingPosition()) {
      // set the time when to dequeue leading agent
      startDequeueTime();
    }
  }

  // let first agent in line pass
  if (dequeueTime <= timeIn) {
    // dequeue agent and inform users
    emit agentMayPass(firstInLine->getId());
    dequeueAgent(firstInLine);
  }
}

void WaitingQueue::onLastAgentPositionChanged(double xIn, double yIn) {
  emit queueEndPositionChanged(xIn, yIn);
}

Ped::Tangle WaitingQueue::getDirection() const { return direction; }

void WaitingQueue::setDirection(const Ped::Tangle& angleIn) {
  direction = angleIn;

  // inform users
  emit directionChanged(direction.toRadian());
}

void WaitingQueue::setDirection(double xIn, double yIn) {
  setDirection(Ped::Tvector(xIn, yIn));
}

void WaitingQueue::setDirection(const Ped::Tvector& directionIn) {
  direction = directionIn.polarAngle();

  // inform users
  emit directionChanged(direction.toRadian());
}

bool WaitingQueue::isEmpty() const { return queuedAgents.isEmpty(); }

Ped::Tvector WaitingQueue::getQueueEndPosition() const {
  if (queuedAgents.isEmpty())
    return position;
  else
    return queuedAgents.last()->getPosition();
}

const Agent* WaitingQueue::enqueueAgent(Agent* agentIn) {
  // determine output
  const Agent* aheadAgent =
      (queuedAgents.isEmpty()) ? nullptr : queuedAgents.last();

  // add agent to queue
  queuedAgents.append(agentIn);

  // inform about new first in line
  if (aheadAgent == nullptr) {
    emit queueLeaderChanged(agentIn->getId());
  }

  // stay informed about updates on queue end
  connect(agentIn, SIGNAL(positionChanged(double, double)), this,
          SLOT(onLastAgentPositionChanged(double, double)));
  // ignore updates from previous queue end
  if (aheadAgent != nullptr) {
    disconnect(aheadAgent, SIGNAL(positionChanged(double, double)), this,
               SLOT(onLastAgentPositionChanged(double, double)));
  }

  // inform users
  emit queueEndChanged();
  informAboutEndPosition();

  // return agent ahead of the new agent
  return aheadAgent;
}

bool WaitingQueue::dequeueAgent(Agent* agentIn) {
  // sanity checks
  if (queuedAgents.isEmpty()) {
    ROS_DEBUG("Cannot dequeue agent from empty waiting queue!");
    return false;
  }

  // remove agent from queue
  bool dequeueSuccess;
  bool dequeuedWasFirst = (queuedAgents.first() == agentIn);
  bool dequeuedWasLast = (queuedAgents.last() == agentIn);
  if (dequeuedWasFirst) {
    queuedAgents.removeFirst();
    dequeueSuccess = true;
  } else {
    ROS_DEBUG("Dequeueing agent from queue (%s), not in front of the queue",
              agentIn->toString().toStdString().c_str());
    int removedCount = queuedAgents.removeAll(agentIn);
    dequeueSuccess = (removedCount >= 1);

    if (dequeueSuccess == false) {
      ROS_DEBUG("Agent isn't waiting in queue! (Agent: %s, Queue: %s)",
                agentIn->toString().toStdString().c_str(),
                this->toString().toStdString().c_str());
      return false;
    }
  }

  // inform other agents
  emit agentDequeued(agentIn->getId());

  // update leading position
  if (dequeuedWasFirst) {
    // determine new first agent in line
    const Agent* newFront =
        (queuedAgents.isEmpty()) ? nullptr : queuedAgents.first();

    // reset time for next agent
    resetDequeueTime();

    // inform users about changed front position
    int frontId = (newFront != nullptr) ? newFront->getId() : -1;
    emit queueLeaderChanged(frontId);
  }

  // update queue end
  if (dequeuedWasLast) {
    disconnect(agentIn, SIGNAL(positionChanged(double, double)), this,
               SLOT(onLastAgentPositionChanged(double, double)));

    emit queueEndChanged();
    informAboutEndPosition();
  }

  return dequeueSuccess;
}

bool WaitingQueue::hasReachedWaitingPosition() {
  if (queuedAgents.isEmpty()) return false;

  // const double waitingRadius = 0.7;
  const double waitingRadius = 0.3;

  // compute distance from where queue starts
  const Agent* leadingAgent = queuedAgents.first();
  Ped::Tvector diff = leadingAgent->getPosition() - position;
  return (diff.length() < waitingRadius);
}

void WaitingQueue::resetDequeueTime() { dequeueTime = INFINITY; }

void WaitingQueue::startDequeueTime() {
  // draw random waiting period
  // exponential_distribution<> distribution ( waitDurationLambda );

  // construct an Erlang distribution from a Gamma
  const int alpha = 2;
  const double beta = 0.5;
  gamma_distribution<> distribution(alpha, beta);

  double waitDuration = distribution(RNG());
  dequeueTime = SCENE.getTime() + waitDuration;
}

void WaitingQueue::informAboutEndPosition() {
  // inform users
  if (queuedAgents.isEmpty()) {
    emit queueEndPositionChanged(position.x, position.y);
  } else {
    Agent* lastAgent = queuedAgents.last();
    Ped::Tvector endPosition = lastAgent->getPosition();
    emit queueEndPositionChanged(endPosition.x, endPosition.y);
  }
}

Ped::Tvector WaitingQueue::closestPoint(const Ped::Tvector& p,
                                        bool* withinWaypoint) const {
  return getQueueEndPosition();
}

QPointF WaitingQueue::getVisiblePosition() const {
  return QPointF(position.x, position.y);
}

void WaitingQueue::setVisiblePosition(const QPointF& positionIn) {
  setPosition(positionIn.x(), positionIn.y());
}

QString WaitingQueue::toString() const {
  QStringList waitingIDs;
  foreach (const Agent* agent, queuedAgents)
    waitingIDs.append(QString::number(agent->getId()));
  QString waitingString = waitingIDs.join(",");

  return tr("WaitingQueue '%1' (@%2,%3; queue: %4)")
      .arg(name)
      .arg(position.x)
      .arg(position.y)
      .arg(waitingString);
}

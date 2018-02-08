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

#include <pedsim/ped_agent.h>
#include <pedsim_simulator/element/queueingwaypoint.h>

QueueingWaypoint::QueueingWaypoint(const QString& nameIn,
                                   const Ped::Tvector& positionIn)
    : Waypoint(nameIn, positionIn) {}

QueueingWaypoint::~QueueingWaypoint() {}

QString QueueingWaypoint::getName() const { return name; }

Ped::Tvector QueueingWaypoint::getForce(const Ped::Tagent& agentIn,
                                        Ped::Tvector* desiredDirectionOut,
                                        bool* reached) const {
  if (reached != nullptr) *reached = false;

  // compute the force
  Ped::Tvector diff = position - agentIn.getPosition();
  double distance = diff.length();

  const double distanceThreshold = 1.0;
  if (distance >= distanceThreshold) {
    // trivial case: agent is far away
    Ped::Tvector desiredDirection = diff.normalized();
    Ped::Tvector force =
        (desiredDirection * agentIn.getVmax() - agentIn.getVelocity()) /
        agentIn.getRelaxationTime();
    if (desiredDirectionOut != nullptr) {
      *desiredDirectionOut = desiredDirection;
    }
    return force;
  } else {
    // agent is already very close to the waypoint
    Ped::Tvector velocity = agentIn.getVelocity();
    // → decelerate agent
    Ped::Tvector decelerationForce = -velocity / agentIn.getRelaxationTime();

    // → move agent to the correct place
    Ped::Tvector projection = velocity * agentIn.getRelaxationTime();
    Ped::Tvector projectedDiff = diff - projection;
    Ped::Tvector projectionForce = projectedDiff / agentIn.getRelaxationTime();

    Ped::Tvector force = decelerationForce + projectionForce;
    if (desiredDirectionOut != nullptr) {
      *desiredDirectionOut = velocity;
    }
    return force;
  }
}

Ped::Tvector QueueingWaypoint::closestPoint(const Ped::Tvector& posIn,
                                            bool* withinWaypoint) const {
  return position;
}

QPointF QueueingWaypoint::getVisiblePosition() const {
  return QPointF(getx(), gety());
}

void QueueingWaypoint::setVisiblePosition(const QPointF& positionIn) {
  setPosition(positionIn.x(), positionIn.y());
}

QString QueueingWaypoint::toString() const {
  return tr("QueueingWaypoint '%1' (@%2,%3)").arg(name).arg(getx()).arg(gety());
}

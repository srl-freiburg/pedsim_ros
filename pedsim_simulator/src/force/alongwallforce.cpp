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
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/force/alongwallforce.h>
#include <pedsim_simulator/scene.h>

#include <ros/ros.h>

AlongWallForce::AlongWallForce(Agent* agentIn) : Force(agentIn) {
  // initialize values
  // TODO - put these magic values into a yaml parameter file
  speedThreshold = 0.2;
  distanceThreshold = 0.6;
  angleThresholdDegree = 20;
  setFactor(CONFIG.forceAlongWall);

  // connect signals
  connect(&CONFIG, SIGNAL(forceFactorAlongWallChanged(double)), this,
          SLOT(onForceFactorChanged(double)));
}

void AlongWallForce::onForceFactorChanged(double valueIn) {
  setFactor(valueIn);
}

Ped::Tvector AlongWallForce::getForce(Ped::Tvector walkingDirection) {
  if (agent == nullptr) {
    ROS_DEBUG("Cannot compute AlongWallForce for null agent!");
    return Ped::Tvector();
  }

  // check whether the agent is stuck
  // → doesn't move
  if (agent->getVelocity().length() > speedThreshold) return Ped::Tvector();

  // → walks against an obstacle
  Ped::Tvector force;
  Ped::Tvector agentPosition = agent->getPosition();
  const QList<Obstacle*>& obstacles = SCENE.getObstacles();
  // → find closest obstacle
  double minDistance = INFINITY;
  Ped::Tvector minDiff;
  Obstacle* minObstacle = nullptr;
  foreach (Obstacle* currentObstacle, obstacles) {
    Ped::Tvector closestPoint = currentObstacle->closestPoint(agentPosition);
    Ped::Tvector diff = closestPoint - agentPosition;
    double distance = diff.length();
    if (distance < minDistance) {
      minObstacle = currentObstacle;
      minDiff = diff;
      minDistance = distance;
    }
  }

  // check distance to closest obstacle
  if (minDistance > distanceThreshold) return Ped::Tvector();

  // check whether closest point is in walking direction
  const Ped::Tangle angleThreshold =
      Ped::Tangle::fromDegree(angleThresholdDegree);
  Ped::Tangle angle = walkingDirection.angleTo(minDiff);
  if (angle > angleThreshold) return Ped::Tvector();

  ROS_DEBUG("Found Agent %d to be stuck!", agent->getId());

  // set force
  // → project to find walking direction
  Ped::Tvector obstacleDirection =
      minObstacle->getEndPoint() - minObstacle->getStartPoint();
  bool projectionPositive =
      (Ped::Tvector::dotProduct(walkingDirection, obstacleDirection) >= 0);

  Ped::Tvector forceDirection =
      (projectionPositive) ? obstacleDirection : -obstacleDirection;
  forceDirection.normalize();

  // scale force
  force = factor * forceDirection;
  return force;
}

QString AlongWallForce::toString() const {
  return tr("AlongWallForce (factor: %2)").arg(factor);
}

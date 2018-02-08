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

#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/shoppingplanner.h>

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/attractionarea.h>

ShoppingPlanner::ShoppingPlanner() {
  // initialize values
  agent = nullptr;
  currentWaypoint = nullptr;
  attraction = nullptr;
  timeReached = 0;
}

void ShoppingPlanner::loseAttraction() {
  // reset
  delete currentWaypoint;
  currentWaypoint = nullptr;
  attraction = nullptr;
  timeReached = 0;

  // inform users
  emit lostAttraction();
}

bool ShoppingPlanner::setAgent(Agent* agentIn) {
  /// NOTE - should the robot be allowed to shop?
  /// who funds robot's shopping expenses??
  // if (agentIn->getType() == 2)
  // 	return false;

  agent = agentIn;

  // some nice fix to dancing in shops
  agent->disableForce("Social");
  agent->disableForce("Random");
  agent->disableForce("GroupCoherence");
  agent->disableForce("GroupGaze");
  agent->disableForce("GroupRepulsion");

  return true;
}

AttractionArea* ShoppingPlanner::getAttraction() const { return attraction; }

bool ShoppingPlanner::setAttraction(AttractionArea* attractionIn) {
  attraction = attractionIn;

  // reset waypoint
  delete currentWaypoint;
  currentWaypoint = nullptr;

  return true;
}

Waypoint* ShoppingPlanner::getCurrentWaypoint() {
  if (hasCompletedWaypoint()) currentWaypoint = getNextWaypoint();

  return currentWaypoint;
}

bool ShoppingPlanner::hasCompletedWaypoint() {
  if (currentWaypoint == nullptr) return true;

  // check whether agent has reached the waypoint and has been there for a given
  // time
  const double distanceThreshold = 1.0;
  // TODO - make shopping time also random
  const double waitTime = 15.0;
  double distance =
      (agent->getPosition() - currentWaypoint->getPosition()).length();
  if (distance <= distanceThreshold) {
    double sceneTime = SCENE.getTime();
    if (timeReached == 0)
      timeReached = sceneTime;
    else if (timeReached - sceneTime >= waitTime) {
      return true;
    }
  }

  return false;
}

bool ShoppingPlanner::hasCompletedDestination() const {
  // Note: The shopping planner is never done.
  //       Change Planner via StateMachine!
  return false;
}

Waypoint* ShoppingPlanner::getNextWaypoint() {
  bool hadWaypoint = (currentWaypoint != nullptr);
  Waypoint* oldWaypoint = currentWaypoint;

  // set new waypoint
  // TODO: create random attraction point in attraction
  QString name = createWaypointName();
  Ped::Tvector position;
  if (!hadWaypoint) {
    // TODO: closest point or random point?
    // maybe also add some timing here, only change position after some random
    // wait (Erlang dist)
    position = getRandomAttractionPosition();
  } else {
    position = oldWaypoint->getPosition();
    // → add random offset
    position += createRandomOffset();
  }

  // → ensure that the position is within the area
  QRectF area(QPointF(0, 0), attraction->getSize());
  area.moveCenter(
      QPointF(attraction->getPosition().x, attraction->getPosition().y));
  position.x = qBound(area.left(), position.x, area.right());
  position.y = qBound(area.top(), position.y, area.bottom());

  // → create new waypoint
  currentWaypoint = new AreaWaypoint(name, position, 0.5);

  // reset reached time
  timeReached = 0;

  // remove previous waypoint
  delete oldWaypoint;
  oldWaypoint = nullptr;

  return currentWaypoint;
}

QString ShoppingPlanner::createWaypointName() const {
  return QString("AttractionHelper_A%1_Q%2")
      .arg(agent->getId())
      .arg(attraction->getName());
}

Ped::Tvector ShoppingPlanner::getRandomAttractionPosition() const {
  Ped::Tvector randomPosition = attraction->getPosition();

  // → add random part
  QSizeF size = attraction->getSize();
  std::uniform_real_distribution<double> xDistribution(-size.width() / 2,
                                                       size.width() / 2);
  std::uniform_real_distribution<double> yDistribution(-size.height() / 2,
                                                       size.height() / 2);

  double xdiff = xDistribution(RNG());
  double ydiff = yDistribution(RNG());

  randomPosition += Ped::Tvector(xdiff, ydiff);

  return randomPosition;
}

Ped::Tvector ShoppingPlanner::createRandomOffset() const {
  const double radiusStd = 4;
  std::normal_distribution<double> radiusDistribution(0, radiusStd);
  double radius = radiusDistribution(RNG());

  std::discrete_distribution<int> angleDistribution{0,   45,  90,  135, 180,
                                                    225, 270, 315, 360};
  double angle = angleDistribution(RNG());

  Ped::Tvector randomOffset =
      Ped::Tvector::fromPolar(Ped::Tangle::fromDegree(angle), radius);

  // only update for significant shopping idea change
  if (randomOffset.lengthSquared() < 2.0) return Ped::Tvector(0, 0, 0);

  return randomOffset;
}

QString ShoppingPlanner::name() const { return tr("ShoppingPlanner"); }

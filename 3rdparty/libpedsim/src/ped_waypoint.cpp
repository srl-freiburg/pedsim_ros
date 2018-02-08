//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#include "ped_waypoint.h"
#include "ped_agent.h"

// initialize static variables
int Ped::Twaypoint::staticid = 0;

/// Constructor: Sets some initial values.
/// \param   xIn The x coordinate of the waypoint
/// \param   yIn The y coordinate of the waypoint
Ped::Twaypoint::Twaypoint(double xIn, double yIn)
    : id(staticid++), position(xIn, yIn), type(Ped::Twaypoint::AreaWaypoint) {}

/// Constructor: Sets some intial values.
/// \param   posIn The position of the waypoint
Ped::Twaypoint::Twaypoint(const Ped::Tvector& posIn)
    : id(staticid++), position(posIn), type(Ped::Twaypoint::AreaWaypoint) {}

/// Destructor
/// \author  chgloor
Ped::Twaypoint::~Twaypoint() {}

/// Returns the force into the direction of the waypoint
/// \param   agentPos The current position of the agent
/// \param   *reached Set to true if the agent has reached the waypoint in this
/// call.
/// \return  Tvector The calculated force
Ped::Tvector Ped::Twaypoint::getForce(const Ped::Tagent& agent,
                                      Ped::Tvector* desiredDirectionOut,
                                      bool* reachedOut) const {
  // set default output parameters
  if (reachedOut != NULL) *reachedOut = false;
  if (desiredDirectionOut != NULL) *desiredDirectionOut = Ped::Tvector();

  Ped::Tvector agentPos = agent.getPosition();
  Ped::Tvector destination = closestPoint(agentPos, reachedOut);
  Ped::Tvector diff = destination - agentPos;

  Ped::Tvector desiredDirection = diff.normalized();
  Tvector force = (desiredDirection * agent.getVmax() - agent.getVelocity()) /
                  agent.getRelaxationTime();

  if (desiredDirectionOut != NULL) *desiredDirectionOut = desiredDirection;

  return force;
}

Ped::Tvector Ped::Twaypoint::closestPoint(const Ped::Tvector& p,
                                          bool* withinWaypoint) const {
  return position;
}

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

#include <pedsim_simulator/element/agent.hpp>
#include <pedsim_simulator/element/areawaypoint.hpp>
#include <pedsim_simulator/element/waitingqueue.hpp>
#include <pedsim_simulator/waypointplanner/individualwaypointplanner.hpp>

IndividualWaypointPlanner::IndividualWaypointPlanner() {
  // initialize values
  agent = nullptr;
  destination = nullptr;
}

bool IndividualWaypointPlanner::setAgent(Agent *agentIn) {
  agent = agentIn;
  return true;
}

Waypoint *IndividualWaypointPlanner::getDestination() const {
  return destination;
}

void IndividualWaypointPlanner::setDestination(Waypoint *waypointIn) {
  destination = waypointIn;
}

Waypoint *IndividualWaypointPlanner::getCurrentWaypoint() {
  return destination;
}

bool IndividualWaypointPlanner::hasCompletedDestination() const {
  if (destination == nullptr) {
    RCLCPP_DEBUG(rclcpp::get_logger(""),
                 "IndividualWaypointPlanner: No destination set!");
    return true;
  }

  // check whether group has reached waypoint
  AreaWaypoint *areaWaypoint = dynamic_cast<AreaWaypoint *>(destination);
  if (areaWaypoint != nullptr) {
    return areaWaypoint->isWithinArea(agent->getPosition());
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger(""), "Unknown Waypoint type: %s",
                 destination->toString().toStdString().c_str());
    return true;
  }
}

QString IndividualWaypointPlanner::name() const {
  return tr("IndividualWaypointPlanner");
}

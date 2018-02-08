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
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/waypointplanner/groupwaypointplanner.h>

GroupWaypointPlanner::GroupWaypointPlanner() {
  // initialize values
  group = nullptr;
  destination = nullptr;
}

bool GroupWaypointPlanner::setGroup(AgentGroup* groupIn) {
  group = groupIn;

  return true;
}

Waypoint* GroupWaypointPlanner::getDestination() const { return destination; }

void GroupWaypointPlanner::setDestination(Waypoint* waypointIn) {
  destination = waypointIn;
}

Waypoint* GroupWaypointPlanner::getCurrentWaypoint() { return destination; }

bool GroupWaypointPlanner::hasCompletedDestination() const {
  if (destination == nullptr) {
    ROS_DEBUG("GroupWaypointPlanner: No destination set!");
    return true;
  }

  // check whether group has reached waypoint
  Ped::Tvector com = group->getCenterOfMass();
  AreaWaypoint* areaWaypoint = dynamic_cast<AreaWaypoint*>(destination);
  if (areaWaypoint != nullptr) {
    return areaWaypoint->isWithinArea(com);
  } else {
    ROS_DEBUG("Unknown Waypoint type: %s",
              destination->toString().toStdString().c_str());
    return true;
  }
}

QString GroupWaypointPlanner::name() const {
  return tr("GroupWaypointPlanner");
}

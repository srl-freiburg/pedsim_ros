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
#include <pedsim_simulator/force/grouprepulsionforce.h>

#include <ros/ros.h>

GroupRepulsionForce::GroupRepulsionForce(Agent* agentIn) : Force(agentIn) {
  // initialize values
  setFactor(CONFIG.forceGroupRepulsion);
  overlapDistance = 0.5;

  // connect signals
  connect(&CONFIG, SIGNAL(forceFactorGroupRepulsionChanged(double)), this,
          SLOT(onForceFactorGroupRepulsionChanged(double)));
}

void GroupRepulsionForce::onForceFactorGroupRepulsionChanged(double valueIn) {
  setFactor(valueIn);
}

void GroupRepulsionForce::setGroup(AgentGroup* groupIn) { group = groupIn; }

const AgentGroup& GroupRepulsionForce::getGroup() const { return *group; }

Ped::Tvector GroupRepulsionForce::getForce(Ped::Tvector walkingDirection) {
  // sanity checks
  if (group->isEmpty()) {
    ROS_DEBUG("Computing GroupRepulsionForce for empty group!");
    return Ped::Tvector();
  }

  // compute group repulsion force
  Ped::Tvector force;
  // → iterate over all group members
  foreach (Agent* currentAgent, group->getMembers()) {
    // → we don't need to take the our agent into account
    if (agent == currentAgent) continue;

    // → compute relative distance vector
    Ped::Tvector diff = agent->getPosition() - currentAgent->getPosition();
    double distance = diff.length();

    // → check whether other agent is overlapping
    if (distance < overlapDistance) force += diff;
  }

  // there is no factor for myForce, hence we have to do it
  force *= factor;

  return force;
}

QString GroupRepulsionForce::toString() const {
  return tr("GroupRepulsionForce (factor: %1)").arg(factor);
}

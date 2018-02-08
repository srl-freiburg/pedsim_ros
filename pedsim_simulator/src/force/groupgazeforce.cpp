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
#include <pedsim_simulator/force/groupgazeforce.h>

#include <ros/ros.h>

GroupGazeForce::GroupGazeForce(Agent* agentIn) : Force(agentIn) {
  // initialize values
  setFactor(CONFIG.forceGroupGaze);
  usePaperVersion = true;

  // connect signals
  connect(&CONFIG, SIGNAL(forceFactorGroupGazeChanged(double)), this,
          SLOT(onForceFactorGroupGazeChanged(double)));
}

void GroupGazeForce::onForceFactorGroupGazeChanged(double valueIn) {
  setFactor(valueIn);
}

void GroupGazeForce::setGroup(AgentGroup* groupIn) { group = groupIn; }

const AgentGroup& GroupGazeForce::getGroup() const { return *group; }

Ped::Tvector GroupGazeForce::getForce(Ped::Tvector walkingDirection) {
  // sanity checks
  if (group->isEmpty()) {
    ROS_DEBUG("Computing GroupGazeForce for empty group!");
    return Ped::Tvector();
  }

  // 1-agent groups don't need to compute this
  // TODO: maybe we shouldn't add this behavior to such groups
  int memberCount = group->memberCount();
  if (memberCount <= 1) return Ped::Tvector();

  // compute group force
  Ped::Tvector com = group->getCenterOfMass();
  // → use center of mass without the current agent
  com = (1 / (double)(memberCount - 1)) *
        (memberCount * com - agent->getPosition());

  // compute relative position of center of mass
  Ped::Tvector relativeCoM = com - agent->getPosition();

  // ensure that center of mass is in angle of vision (phi) in radians
  Ped::Tangle visionAngle = Ped::Tangle::fromDegree(90);
  // → angle between walking direction and center of mass
  // TODO: move this to a generic place
  double elementProduct =
      Ped::Tvector::dotProduct(walkingDirection, relativeCoM);
  // note: acos() returns the not directed angle in [0, pi]
  Ped::Tangle comAngle = Ped::Tangle::fromRadian(acos(
      elementProduct / (walkingDirection.length() * relativeCoM.length())));

  // compute force
  // → compute gazing direction
  if (comAngle > visionAngle) {
    Ped::Tvector force;

    // → switch between definitions
    if (usePaperVersion) {
      // agent has to rotate
      Ped::Tangle necessaryRotation = comAngle - visionAngle;

      force = -necessaryRotation.toRadian() * walkingDirection;
    } else {
      // agent has to rotate
      // HACK: this isn't the specification of the Moussaid et al. paper!
      //      but it decreases the amount of abrupt changes
      //      problem: it more or less hard codes that agents want to stay in
      //      line
      double walkingDirectionSquared = walkingDirection.lengthSquared();
      double walkingDirectionDistance =
          elementProduct / walkingDirectionSquared;
      force = walkingDirectionDistance * walkingDirection;
    }

    // there is no factor for myForce, hence we have to do it
    force *= factor;

    return force;
  } else {
    // no force
    return Ped::Tvector();
  }
}

QString GroupGazeForce::toString() const {
  return tr("GroupGazeForce (factor: %1)").arg(factor);
}

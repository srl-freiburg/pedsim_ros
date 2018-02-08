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
#include <pedsim_simulator/force/groupcoherenceforce.h>

#include <ros/ros.h>

GroupCoherenceForce::GroupCoherenceForce(Agent* agentIn) : Force(agentIn) {
  // initialize values
  setFactor(CONFIG.forceGroupCoherence);
  usePaperVersion = true;

  // connect signals
  connect(&CONFIG, SIGNAL(forceFactorGroupCoherenceChanged(double)), this,
          SLOT(onForceFactorGroupCoherenceChanged(double)));
}

void GroupCoherenceForce::onForceFactorGroupCoherenceChanged(double valueIn) {
  setFactor(valueIn);
}

void GroupCoherenceForce::setGroup(AgentGroup* groupIn) { group = groupIn; }

const AgentGroup& GroupCoherenceForce::getGroup() const { return *group; }

Ped::Tvector GroupCoherenceForce::getForce(Ped::Tvector walkingDirection) {
  // sanity checks
  if (group->isEmpty()) {
    ROS_DEBUG("Computing GroupCoherenceForce for empty group!");
    return Ped::Tvector();
  }

  // compute group coherence force
  // → compute relative position of center of mass
  Ped::Tvector com = group->getCenterOfMass();
  Ped::Tvector relativeCoM = com - agent->getPosition();
  // → distance to center of mass
  double distance = relativeCoM.length();
  // → approximate maximal distance, according to paper
  const double maxDistance = ((double)group->memberCount() - 1) / 2;

  // compute force
  Ped::Tvector force;
  // → switch between definitions
  if (usePaperVersion) {
    // force according to paper
    // → check whether maximal distance has been exceeded
    if (distance >= maxDistance) {
      // compute force
      force = relativeCoM.normalized();

      // there is no factor for myForce, hence we have to do it
      force *= factor;

      return force;
    } else {
      // there is no force
      return Ped::Tvector();
    }
  } else {
    // modified force
    force = relativeCoM;

    // there is no factor for myForce, hence we have to do it
    // HACK: use smooth transition
    //      this doesn't follow the Moussaid paper, but it creates less abrupt
    //      changes
    double softenedFactor = factor * (tanh(distance - maxDistance) + 1) / 2;
    force *= softenedFactor;

    ROS_DEBUG("softenedFactor = %f = %f * (tanh(%f - %f)+1) / 2",
              softenedFactor, factor, distance, maxDistance);

    return force;
  }
}

QString GroupCoherenceForce::toString() const {
  return tr("GroupCoherenceForce (factor: %1)").arg(factor);
}

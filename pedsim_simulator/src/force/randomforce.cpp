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
#include <pedsim_simulator/force/randomforce.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>

#include <ros/ros.h>

RandomForce::RandomForce(Agent* agentIn) : Force(agentIn) {
  // initialize values
  setFactor(CONFIG.forceRandom);
  fadingDuration = 1;
  nextDeviation = computeNewDeviation();

  // connect signals
  connect(&CONFIG, SIGNAL(forceFactorRandomChanged(double)), this,
          SLOT(onForceFactorChanged(double)));
}

void RandomForce::onForceFactorChanged(double valueIn) { setFactor(valueIn); }

void RandomForce::setFadingTime(double durationIn) {
  // sanity checks
  if (durationIn < 0) {
    ROS_DEBUG("Cannot set fading time to invalid value: %f", durationIn);
    return;
  }

  fadingDuration = durationIn;
}

double RandomForce::getFadingTime() const { return fadingDuration; }

Ped::Tvector RandomForce::computeNewDeviation() {
  // set up random distributions
  uniform_real_distribution<double> angleDistribution(0, 360);
  double deviationAngle = angleDistribution(RNG());
  normal_distribution<double> distanceDistribution(0, 1);
  double deviationDistance = distanceDistribution(RNG());

  // create deviation from polar coordinates
  Ped::Tvector deviation = Ped::Tvector::fromPolar(
      Ped::Tangle::fromDegree(deviationAngle), deviationDistance);
  return deviation;
}

Ped::Tvector RandomForce::getForce(Ped::Tvector walkingDirection) {
  // use the current time to compute the fading progress
  double time = SCENE.getTime();
  double progress = fmod(time, fadingDuration);

  // create a new fading goal when necessary
  if (progress < CONFIG.getTimeStepSize()) {
    lastDeviation = nextDeviation;
    nextDeviation = computeNewDeviation();
  }

  // compute the force
  Ped::Tvector force =
      (1 - progress) * lastDeviation + progress * nextDeviation;

  // scale force
  force *= factor;

  return force;
}

QString RandomForce::toString() const {
  return tr("RandomForce (fading duration: %1; factor: %2)")
      .arg(fadingDuration)
      .arg(factor);
}

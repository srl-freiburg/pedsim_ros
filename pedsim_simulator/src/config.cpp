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

// initialize static value
Config* Config::Config::instance = nullptr;

Config::Config(QObject* parent) {
  updateRate = 25.0;
  simulationFactor = 1.0;

  forceObstacle = 10.0;
  sigmaObstacle = 0.2;
  forceSocial = 5.1;

  forceGroupGaze = 3.0;
  forceGroupCoherence = 2.0;
  forceGroupRepulsion = 1.0;
  forceRandom = 0.1;
  forceAlongWall = 2.0;

  cell_width = 1.0;
  cell_height = 1.0;

  robot_mode = RobotMode::TELEOPERATION;
  robot_wait_time = 15;
  max_robot_speed = 2.0;

  groups_enabled = true;
  group_size_lambda = 1.1;
  wait_time_beta = 0.2;

  visual_mode = VisualMode::MINIMAL;
}

Config& Config::getInstance() {
  if (instance == nullptr) instance = new Config();

  return *instance;
}

void Config::setObstacleForce(double valueIn) {
  forceObstacle = valueIn;

  // inform users
  emit forceFactorChanged("obstacle", valueIn);
  emit forceFactorObstacleChanged(valueIn);
}

void Config::setObstacleSigma(double valueIn) {
  sigmaObstacle = valueIn;

  // inform users
  emit forceFactorChanged("obstacle_sigma", valueIn);
  emit forceSigmaObstacleChanged(valueIn);
}

void Config::setSocialForce(double valueIn) {
  forceSocial = valueIn;

  // inform users
  emit forceFactorChanged("social", valueIn);
  emit forceFactorSocialChanged(valueIn);
}

void Config::setGroupGazeForce(double valueIn) {
  forceGroupGaze = valueIn;

  // inform users
  emit forceFactorChanged("group_gaze", valueIn);
  emit forceFactorGroupGazeChanged(valueIn);
}

void Config::setGroupCoherenceForce(double valueIn) {
  forceGroupCoherence = valueIn;

  // inform users
  emit forceFactorChanged("group_coherence", valueIn);
  emit forceFactorGroupCoherenceChanged(valueIn);
}

void Config::setGroupRepulsionForce(double valueIn) {
  forceGroupRepulsion = valueIn;

  // inform users
  emit forceFactorChanged("group_repulsion", valueIn);
  emit forceFactorGroupRepulsionChanged(valueIn);
}

void Config::setRandomForce(double valueIn) {
  forceRandom = valueIn;

  // inform users
  emit forceFactorChanged("random", valueIn);
  emit forceFactorRandomChanged(valueIn);
}

void Config::setAlongWallForce(double valueIn) {
  forceAlongWall = valueIn;

  // inform users
  emit forceFactorChanged("alongwall", valueIn);
  emit forceFactorAlongWallChanged(valueIn);
}

QMap<QString, double> Config::getForceMap() const {
  // create output map
  QMap<QString, double> forceMap;
  // → fill map
  forceMap["obstacle"] = forceObstacle;
  forceMap["obstacle_sigma"] = sigmaObstacle;
  forceMap["social"] = forceSocial;
  forceMap["group_gaze"] = forceGroupGaze;
  forceMap["group_coherence"] = forceGroupCoherence;
  forceMap["group_repulsion"] = forceGroupRepulsion;
  forceMap["random"] = forceRandom;
  forceMap["alongwall"] = forceAlongWall;

  return forceMap;
}

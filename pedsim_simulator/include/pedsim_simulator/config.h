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

#ifndef _config_h_
#define _config_h_

#include <QMap>
#include <QObject>

#include <pedsim_simulator/utilities.h>

class Config : public QObject {
  Q_OBJECT

 protected:
  Config(QObject* parent = 0);

// Singleton Design Pattern
#define CONFIG Config::getInstance()
 protected:
  static Config* instance;

 public:
  static Config& getInstance();

  // Signals
 signals:
  // → Forces
  void forceFactorChanged(QString name, double value);
  void forceFactorObstacleChanged(double value);
  void forceSigmaObstacleChanged(double value);
  void forceFactorSocialChanged(double value);
  void forceFactorGroupGazeChanged(double value);
  void forceFactorGroupCoherenceChanged(double value);
  void forceFactorGroupRepulsionChanged(double value);
  void forceFactorRandomChanged(double value);
  void forceFactorAlongWallChanged(double value);

  // Slots
 public slots:
  // → Forces
  void setObstacleForce(double valueIn);
  void setObstacleSigma(double valueIn);
  void setSocialForce(double valueIn);
  void setGroupGazeForce(double valueIn);
  void setGroupCoherenceForce(double valueIn);
  void setGroupRepulsionForce(double valueIn);
  void setRandomForce(double valueIn);
  void setAlongWallForce(double valueIn);

  // Methods
 public:
  // TODO - change to std::unordered_map
  QMap<QString, double> getForceMap() const;

  double getTimeStepSize() { return simulationFactor / updateRate; }

  // Attributes
 public:
  // Simulation
  double updateRate;
  double simulationFactor;

  // Forces
  double forceObstacle;
  double sigmaObstacle;
  double forceSocial;
  double forceGroupGaze;
  double forceGroupCoherence;
  double forceGroupRepulsion;
  double forceRandom;
  double forceAlongWall;

  // robot control
  RobotMode robot_mode;
  int robot_wait_time;
  double max_robot_speed;

  // enable/disable groups behaviour
  bool groups_enabled;

  // cells
  double cell_width;
  double cell_height;

  // distribution parameters
  double group_size_lambda;
  double wait_time_beta;

  // simulation visualization mode
  VisualMode visual_mode;
};

#endif

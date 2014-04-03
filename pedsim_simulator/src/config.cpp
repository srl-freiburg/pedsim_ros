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

// Includes
#include <pedsim_simulator/config.h>
// → Qt
#include <QSettings>
#include <QStringList>


// initialize static value
Config* Config::Config::instance = nullptr;


Config::Config(QObject* parent) {
	// load configuration
	// TODO - remove these
	QSettings settings;
// 	showWaypoints = settings.value("GUI/ShowWaypoints", false).toBool();
// 	showAttractions = settings.value("GUI/ShowAttractions", true).toBool();
// 	showDirection = settings.value("GUI/ShowDirection", false).toBool();
// 	showAcceleration = settings.value("GUI/ShowAcceleration", false).toBool();
// 	showForces = settings.value("GUI/ShowForces", false).toBool();
// 	showGroups = settings.value("GUI/ShowGroups", true).toBool();
// 	showGrid = settings.value("GUI/ShowGrid", false).toBool();
	
	// → load visibility configuration for individual forces
	QStringList forces({
		"Desired", "Obstacle", "Social",
		"Random", "AlongWall",
		"GroupGaze", "GroupCoherence", "GroupRepulsion"});
	foreach(QString currentForce, forces) {
		QString settingName = QString("ShowForce/%1").arg(currentForce);
		forceVisibilityMap[currentForce] = settings.value(settingName, true).toBool();
	}
	
	timeStepSize = settings.value("Simulation/TimeStepSize", 0.05).toDouble();
	simSpeed = settings.value("Simulation/Speed", round(1000.0/50)).toInt();
	forceObstacle = settings.value("Forces/Obstacle", 10).toDouble();
	sigmaObstacle = settings.value("Forces/ObstacleSigma", 0.2).toDouble();
	forceSocial = settings.value("Forces/Social", 5).toDouble();
	forceGroupGaze = settings.value("Forces/GroupGaze", 3).toDouble();
	forceGroupCoherence = settings.value("Forces/GroupCoherence", 2).toDouble();
	forceGroupRepulsion = settings.value("Forces/GroupRepulsion", 1).toDouble();
	forceRandom = settings.value("Forces/Random", 0.1).toDouble();
	forceAlongWall = settings.value("Forces/AlongWall", 2).toDouble();
	
	// TODO - add utility to switch off group forces and behaviours
}

Config& Config::getInstance() {
	if(instance == nullptr)
		instance = new Config();

	return *instance;
}


void Config::setForceVisibility(const QString& forceIn, bool visibleIn) {
	forceVisibilityMap[forceIn] = visibleIn;

	// save settings
	QSettings settings;
	QString settingName = QString("ShowForce/%1").arg(forceIn);
	settings.setValue(settingName, visibleIn);

	// inform users
	emit forceVisibilityChanged(forceIn, visibleIn);
}

void Config::setTimeStepSize(double valueIn) {
	timeStepSize = valueIn;

	// inform users
	emit timeStepSizeChanged(valueIn);
}

void Config::setSimSpeed(int valueIn) {
	// keep simulation speed in bounds
	simSpeed = qBound(10, valueIn, 1000);

	// inform users
	emit simulationSpeedChanged(simSpeed);
}

void Config::decreaseSimSpeed() {
	setSimSpeed(simSpeed+1);
}

void Config::increaseSimSpeed() {
	setSimSpeed(simSpeed-1);
}

void Config::halveSimSpeed() {
	setSimSpeed(simSpeed*2);
}

void Config::doubleSimSpeed() {
	setSimSpeed(simSpeed/2);
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

QMap<QString,double> Config::getForceMap() const {
	// create output map
	QMap<QString,double> forceMap;
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

bool Config::isForceVisible(const QString& forceNameIn) const {
	return forceVisibilityMap.value(forceNameIn, true);
}

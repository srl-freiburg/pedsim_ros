// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

// Includes
#include <pedsim_simulator/config.h>
// → Qt
#include <QSettings>
#include <QStringList>


// initialize static value
Config* Config::Config::instance = nullptr;


Config::Config(QObject* parent) {
	// load configuration
	QSettings settings;
	showWaypoints = settings.value("GUI/ShowWaypoints", false).toBool();
	showAttractions = settings.value("GUI/ShowAttractions", true).toBool();
	showDirection = settings.value("GUI/ShowDirection", false).toBool();
	showAcceleration = settings.value("GUI/ShowAcceleration", false).toBool();
	showForces = settings.value("GUI/ShowForces", false).toBool();
	showGroups = settings.value("GUI/ShowGroups", true).toBool();
	showGrid = settings.value("GUI/ShowGrid", false).toBool();
	
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

void Config::setShowWaypoints(bool value) {
	showWaypoints = value;

	// save settings
	QSettings settings;
	settings.setValue("GUI/ShowWaypoints", value);

	// inform users
	emit waypointVisibilityChanged(value);
}

void Config::setShowAttractions(bool value) {
	showAttractions = value;

	// save settings
	QSettings settings;
	settings.setValue("GUI/ShowAttractions", value);

	// inform users
	emit attractionVisibilityChanged(value);
}

void Config::setShowDirection(bool valueIn) {
	showDirection = valueIn;

	// save settings
	QSettings settings;
	settings.setValue("GUI/ShowDirection", valueIn);

	// inform users
	emit directionVisibilityChanged(valueIn);
}

void Config::setShowAcceleration(bool valueIn) {
	showAcceleration = valueIn;

	// save settings
	QSettings settings;
	settings.setValue("GUI/ShowAcceleration", valueIn);

	// inform users
	emit accelerationVisibilityChanged(valueIn);
}

void Config::setShowForces(bool valueIn) {
	showForces = valueIn;

	// save settings
	QSettings settings;
	settings.setValue("GUI/ShowForces", valueIn);

	// inform users
	emit forcesVisibilityChanged(valueIn);
}

void Config::setShowGroups(bool valueIn) {
	showGroups = valueIn;

		// save settings
	QSettings settings;
	settings.setValue("GUI/ShowGroups", valueIn);

	// inform users
	emit groupsVisibilityChanged(valueIn);
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

void Config::setShowGrid(bool valueIn) {
	showGrid = valueIn;

	// inform users
	emit gridVisibilityChanged(showGrid);
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

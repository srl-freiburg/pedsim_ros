// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

#ifndef _config_h_
#define _config_h_

// Includes
// → Qt
#include <QObject>
#include <QMap>


class Config : public QObject {
	Q_OBJECT

	// Constructor and Destructor
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
	// → GUI
	void waypointVisibilityChanged(bool show);
	void attractionVisibilityChanged(bool show);
	void directionVisibilityChanged(bool show);
	void accelerationVisibilityChanged(bool show);
	void forcesVisibilityChanged(bool show);
	void groupsVisibilityChanged(bool show);
	void forceVisibilityChanged(QString force, bool show);
	void gridVisibilityChanged(bool show);
	// → Simulation
	void timeStepSizeChanged(double value);
	void simulationSpeedChanged(int value);
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
	void setShowWaypoints(bool valueIn);
	void setShowAttractions(bool valueIn);
	void setShowDirection(bool valueIn);
	void setShowAcceleration(bool valueIn);
	void setShowForces(bool valueIn);
	void setShowGroups(bool valueIn);
	void setForceVisibility(const QString& forceIn, bool visibleIn);
	void setShowGrid(bool valueIn);
	void setSimSpeed(int valueIn);
	void decreaseSimSpeed();
	void increaseSimSpeed();
	void halveSimSpeed();
	void doubleSimSpeed();
	void setTimeStepSize(double valueIn);
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
	QMap<QString,double> getForceMap() const;
	bool isForceVisible(const QString& forceNameIn) const;
	
	//TODO: make member variables protected and add getter functions


	// Attributes
public:
	// → Visualization
	bool showWaypoints;
	bool showAttractions;
	bool showDirection;
	bool showAcceleration;
	bool showForces;
	bool showGroups;
	bool showGrid;
	// → Visibility of individual forces
	QMap<QString,bool> forceVisibilityMap;
	// → Simulation
	double timeStepSize;
	int simSpeed;
	// → Forces
	double forceObstacle;
	double sigmaObstacle;
	double forceSocial;
	double forceGroupGaze;
	double forceGroupCoherence;
	double forceGroupRepulsion;
	double forceRandom;
	double forceAlongWall;
};

#endif

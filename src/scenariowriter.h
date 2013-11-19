// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _scenariowriter_h_
#define _scenariowriter_h_

// Includes
// → SGDiCoP
#include "scene.h"
// → Qt
#include <QObject>
#include <QSharedPointer>


// Forward Declarations
class Scene;


//TODO: Rename to something like ScenarioLoader
class ScenarioWriter : public QObject {
	Q_OBJECT

	// Constructor and Destructor
public:
	ScenarioWriter(const QSharedPointer<Scene>& sceneIn);


	// Methods
public:
	bool writeToFile(const QString& filename);


	// Attributes
private:
	QSharedPointer<Scene> scene;
};

#endif

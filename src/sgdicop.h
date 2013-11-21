// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _sgdicop_h_
#define _sgdicop_h_

// Includes
// → SGDiCoP
#include "config.h"
// → Qt
#include <QSharedPointer>
#include <QThread>

// Forward Declarations
class ConsoleLogger;
class FileLogger;
class MainWindow;
class Scene;


// Macros/Defines
// → Return Values
#define RETURN_OK 0
#define RETURN_ERROR 1


class SGDiCoP : public QThread {
	Q_OBJECT

	// Constructor and Destructor
public:
	SGDiCoP();
	virtual ~SGDiCoP();


    // Singleton Design Pattern
	#define SGDICOP SGDiCoP::getInstance()
protected:
	static SGDiCoP* instance;
public:
	static SGDiCoP& getInstance();


	// Signals
signals:
	void scenarioCreated();
	void scenarioLoaded();
	void scenarioSaved();


	// Methods
public:
	bool initialize(const QStringList& options);
	void quit(int returnValue = RETURN_OK);
	// → pause/unpause
	void pause();
	void unpause();
	bool isPaused() const;
	// → scene handling
	bool clearScene();
	bool newScene();
	bool loadScene(const QString& scenarioFile);


	// Attributes
public:
	QString scenarioFileName;
	QSharedPointer<MainWindow> mainwindow;
	QSharedPointer<Scene> scene;
protected:
	ConsoleLogger* consoleLogger;
	FileLogger* fileLogger;
	bool paused;

	int dummy_argc;
    char** dummy_argv;
};

#endif

// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "sgdicop.h"
// → SGDiCoP
#include "logging.h"
#include "scenarioreader.h"
#include "scene.h"
#include "agent.h"
#include "obstacle.h"
#include "waypoint.h"
#include "mainwindow.h"
// → Qt
#include <QCoreApplication>
#include <QFileInfo>
#include <QMessageBox>


// initialize static value
SGDiCoP* SGDiCoP::SGDiCoP::instance = NULL;


SGDiCoP::SGDiCoP() {
    // set singleton instance
    instance = this;

    // initialize logging
    consoleLogger = new ConsoleLogger();
    fileLogger = new FileLogger("./SGDiCoP.log");

    // start paused
    paused = true;

    dummy_argc = 0;
    char** dd;
    dummy_argv = dd;
}

SGDiCoP::~SGDiCoP() {
    // clean up
    delete fileLogger;
    delete consoleLogger;
}


SGDiCoP& SGDiCoP::getInstance() {
    // there should always be an instance initialized before anything is done
    if(instance == NULL)
        ERROR_LOG("No SGDiCoP instance has been initialized!");

    return *instance;
}


bool SGDiCoP::initialize(const QStringList& options) {
    // create and display GUI
    mainwindow = QSharedPointer<MainWindow>(new MainWindow());
    mainwindow->show();

    
    // create pedsim scene
    ros::init(dummy_argc, dummy_argv, "simulator");
    ros::NodeHandle node;
    scene = QSharedPointer<Scene>(new Scene(mainwindow->graphicsscene, node));

    // load parameters
    std::string scene_file_param;
    node.getParam("/simulator/scene_file", scene_file_param);

    double cell_size;
    node.getParam("/simulator/cell_size", cell_size);
    CONFIG.width = cell_size;
    CONFIG.height = cell_size;


    ROS_INFO("Read parameters (%s) (%f)", scene_file_param.c_str(), cell_size);

    // load scenario file
    QString scenefile = QString::fromStdString(scene_file_param);
    // if(!options.empty() && options.size() >=2 ) {
    //     scenefile = options.first();

    //     QString cf = options[1];
    //     CONFIG.config_file = cf.toStdString();
    //     CONFIG.readParameters(CONFIG.config_file);
    // }
    // else {
    //     // warn user about missing scenario file
    //     QMessageBox::StandardButton buttonPressed = QMessageBox::question(mainwindow.data(),
    //            tr("No input file, config file and weight file specified"),
    //            tr("There hasn't been a input file specified. Do you want to open a file now?"),
    //            QMessageBox::Open|QMessageBox::No,
    //            QMessageBox::Open);

    //     if(buttonPressed == QMessageBox::Open) {
    //         scenefile = mainwindow->showOpenScenarioDialog();
    //     }
    //     else {
    //         // start with an empty environment
    //         INFO_LOG("No scenario file defined, using an empty one.");
    //     }
    // }


    // load scenario from file
    if(!scenefile.isEmpty())
        loadScene(scenefile);

    return true;
}


void SGDiCoP::quit(int returnValue) {
    // exit application
    // -> return from the app.exec() statement within main()
    QCoreApplication::exit(returnValue);

    // -> backup for cases, in which QT's event loop isn't running, yet
    exit(returnValue);
}


void SGDiCoP::pause() {
    //    paused = true;
    paused = false; /// for automation

    // tell the scene to stop updating
    /// commented out for automation
    //    scene->pauseUpdates();
}


void SGDiCoP::unpause() {
    paused = false;

    // tell the scene to resume updating
    scene->unpauseUpdates();
}


bool SGDiCoP::isPaused() const {
    return paused;
}


bool SGDiCoP::clearScene() {
    bool wasPaused = scene->isPaused();

    // stop/block scene update
    if(wasPaused)
        scene->pauseUpdates();

    // update name of scenario file name
    scenarioFileName.clear();

    // recreate the scene
    //HACK: we have to delete and recreate the scene, because libPedSim doesn't
    //      support cleaning
    // 	delete scene;
    // scene = QSharedPointer<Scene>(new Scene(mainwindow->graphicsscene));
    ros::init(dummy_argc, dummy_argv, "simulator");
    ros::NodeHandle node;

    scene = QSharedPointer<Scene>(new Scene(mainwindow->graphicsscene, node));

    // restart scene updates
    if(!wasPaused)
        scene->unpauseUpdates();

    return true;
}

bool SGDiCoP::newScene() {
    bool cleared = clearScene();

    // inform users
    if(cleared)
        emit scenarioLoaded();

    return cleared;
}

bool SGDiCoP::loadScene(const QString& scenarioFile) {
    ScenarioReader scenarioReader(scene);
    bool readResult = scenarioReader.readFromFile(scenarioFile);
    if(readResult == false) {
        ERROR_LOG("Failed to read input file '%1'", scenarioFile);
        return false;
    }

    // update name of scenario file
    scenarioFileName = scenarioFile;
    emit scenarioLoaded();

    return true;
}

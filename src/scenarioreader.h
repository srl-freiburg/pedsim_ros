// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _scenarioreader_h_
#define _scenarioreader_h_

// Includes
// → SGDiCoP
#include "scene.h"
#include "config.h"
// → Qt
#include <QSharedPointer>
#include <QXmlStreamReader>
#include <QPen>
#include <QBrush>
#include <QGraphicsScene>

// Forward Declarations
class Agent;


class ScenarioReader : public QObject {
    Q_OBJECT

    // Constructor and Destructor
public:
    ScenarioReader(const QSharedPointer<Scene>& sceneIn);


    // Methods
public:
    bool readFromFile(const QString& filename);
    void drawObstacles(float x1, float y1, float x2, float y2);

protected:
    void processData(QByteArray& data);


    // Attributes
private:
    QXmlStreamReader xmlReader;
    QSharedPointer<Scene> scene;
    QList<Agent*> currentAgents;
};

#endif

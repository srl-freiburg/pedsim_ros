// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

// Includes
#include <pedsim_simulator/scenarioreader.h>
// → SGDiCoP
// #include "logging.h"
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/element/attractionarea.h>
// → Qt
#include <QFile>

#include <iostream>


ScenarioReader::ScenarioReader() {
	// initialize values
	currentAgents = nullptr;
}


bool ScenarioReader::readFromFile(const QString& filename) {
// 	INFO_LOG("Loading scenario file '%1'.", filename);

	// open file
	QFile file(filename);
	if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
// 		ERROR_LOG("Couldn't open scenario file!");
		return false;
	}

	// read input
	xmlReader.setDevice(&file);

	while(!xmlReader.atEnd()) {
		xmlReader.readNext();
		processData();
	}

	// check for errors
	if(xmlReader.hasError()) {
// 		ERROR_LOG("Error while reading scenario file: %1 (line: %2)", xmlReader.errorString(), xmlReader.lineNumber());
		return false;
	}

	// report success
	return true;
}


void ScenarioReader::processData() {
	if(xmlReader.isStartElement()) {
		const QString elementName = xmlReader.name().toString();
		const QXmlStreamAttributes elementAttributes = xmlReader.attributes();

		if((elementName == "scenario")
			|| (elementName == "welcome")) {
			// nothing to do
		}
		else if(elementName == "obstacle") {
			double x1 = elementAttributes.value("x1").toString().toDouble();
			double y1 = elementAttributes.value("y1").toString().toDouble();
			double x2 = elementAttributes.value("x2").toString().toDouble();
			double y2 = elementAttributes.value("y2").toString().toDouble();
			Obstacle* obs = new Obstacle(x1, y1, x2, y2);
			SCENE.addObstacle(obs);
			SCENE.drawObstacles ( x1, y1, x2, y2 );
		}
		else if(elementName == "waypoint") {
			QString id = elementAttributes.value("id").toString();
			double x = elementAttributes.value("x").toString().toDouble();
			double y = elementAttributes.value("y").toString().toDouble();
			double r = elementAttributes.value("r").toString().toDouble();
			AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
			SCENE.addWaypoint(w);
		}
		else if(elementName == "queue") {
			QString id = elementAttributes.value("id").toString();
			double x = elementAttributes.value("x").toString().toDouble();
			double y = elementAttributes.value("y").toString().toDouble();
			double directionValue = elementAttributes.value("direction").toString().toDouble();

			Ped::Tvector position(x, y);
			Ped::Tangle direction = Ped::Tangle::fromDegree(directionValue);

			WaitingQueue* queue = new WaitingQueue(id, position, direction);
			SCENE.addWaitingQueue(queue);
		}
		else if(elementName == "attraction") {
			QString id = elementAttributes.value("id").toString();
			double x = elementAttributes.value("x").toString().toDouble();
			double y = elementAttributes.value("y").toString().toDouble();
			double width = elementAttributes.value("width").toString().toDouble();
			double height = elementAttributes.value("height").toString().toDouble();
			double strength = elementAttributes.value("strength").toString().toDouble();

			AttractionArea* attraction = new AttractionArea(id);
			attraction->setPosition(x, y);
			attraction->setSize(width, height);
			attraction->setStrength(strength);
			SCENE.addAttraction(attraction);
		}
		else if(elementName == "agent") {
			double x = elementAttributes.value("x").toString().toDouble();
			double y = elementAttributes.value("y").toString().toDouble();
			int n = elementAttributes.value("n").toString().toInt();
			double dx = elementAttributes.value("dx").toString().toDouble();
			double dy = elementAttributes.value("dy").toString().toDouble();
			int type = elementAttributes.value("type").toString().toInt();
			AgentCluster* agentCluster = new AgentCluster(x, y, n);
			agentCluster->setDistribution(dx, dy);
			agentCluster->setType(type);
			SCENE.addAgentCluster(agentCluster);
			currentAgents = agentCluster;

			std::cout << "Added agent cluster size " << n << std::endl;
		}
		// → agent's inner elements
		else if(elementName == "addwaypoint") {
			if(currentAgents == nullptr) {
// 				WARN_LOG("Invalid <addwaypoint> element outside of agent element!");
				return;
			}

			// add waypoints to current <agent> element
			QString id = elementAttributes.value("id").toString();
			currentAgents->addWaypoint(SCENE.getWaypointByName(id));
		}
		else if(elementName == "addqueue") {
			if(currentAgents == nullptr) {
// 				WARN_LOG("Invalid <addqueue> element outside of agent element!");
				return;
			}

			// add waiting queue to current <agent> element
			QString id = elementAttributes.value("id").toString();
			currentAgents->addWaitingQueue(SCENE.getWaitingQueueByName(id));
		}
		else {
			// inform the user about invalid elements
// 			ERROR_LOG("Unknown element: <%1>", elementName);
		}
	}
	else if(xmlReader.isEndElement()) {
		const QString elementName = xmlReader.name().toString();

		if(elementName == "agent") {
			currentAgents = nullptr;
		}
	}
}

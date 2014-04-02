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
#include <pedsim_simulator/scene.h>
// → Qt
#include <QPair>
#include <QXmlStreamReader>


class ScenarioReader {
	// Constructor and Destructor
public:
	ScenarioReader(); 


	// Methods
public:
	bool readFromFile(const QString& filename);

protected:
	void processData();


	// Attributes
private:
	QXmlStreamReader xmlReader;

	AgentCluster* currentAgents;
};

#endif

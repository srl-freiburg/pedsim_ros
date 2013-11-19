// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include "scenariowriter.h"
// → SGDiCoP
#include "agent.h"
#include "obstacle.h"
#include "waypoint.h"
#include "logging.h"
// → Qt
#include <QFile>
#include <QXmlStreamReader>


ScenarioWriter::ScenarioWriter(const QSharedPointer<Scene>& sceneIn) {
	scene = sceneIn;
}


/// Called for each line in the file
/// \date    2012-02-03
bool ScenarioWriter::writeToFile(const QString& filename) {
	INFO_LOG("Writing scenario file '%1'.", filename);

	// open output file
	QFile file(filename);
	if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		ERROR_LOG("Couldn't open '%1' to write scenario file!", filename);
		return false;
	}

	// start writing file
	QXmlStreamWriter xmlWriter(&file);
	xmlWriter.setAutoFormatting(true);
	xmlWriter.writeStartDocument();

	// write something about the scenario
	QDateTime datetime = QDateTime::currentDateTime();
	QString introComment = QString("This scenario file was created by SGDiCoP on %1").arg(datetime.toString(Qt::ISODate));
	xmlWriter.writeComment(introComment);

	// start scenario tag
	xmlWriter.writeStartElement("scenario");

	// write obstacles
	xmlWriter.writeComment("Obstacles");
	foreach(Obstacle* obs, scene->obstacles) {
		xmlWriter.writeStartElement("obstacle");
		xmlWriter.writeAttribute("x1", QString::number(obs->getax()));
		xmlWriter.writeAttribute("y1", QString::number(obs->getay()));
		xmlWriter.writeAttribute("x2", QString::number(obs->getbx()));
		xmlWriter.writeAttribute("y2", QString::number(obs->getby()));
		xmlWriter.writeEndElement();
	}

	// write waypoints
	xmlWriter.writeComment("Waypoints");
	QMap<QString, Waypoint*>::const_iterator iter = scene->waypoints.constBegin();
	while(iter != scene->waypoints.constEnd()) {
		xmlWriter.writeStartElement("waypoint");
		QString id = iter.key();
		Waypoint* waypoint = iter.value();
		//TODO: libPedSim doesn't handle the id within Twaypoint, and the code wants to use an int instead of a string
		xmlWriter.writeAttribute("id", id);
		xmlWriter.writeAttribute("x", QString::number(waypoint->getx()));
		xmlWriter.writeAttribute("y", QString::number(waypoint->gety()));
		xmlWriter.writeAttribute("r", QString::number(waypoint->getr()));
		//TODO: type?!?
		xmlWriter.writeEndElement();
		++iter;
	}

	// write agents
	xmlWriter.writeComment("Agents");
	foreach(Agent* agent, scene->agents) {
		xmlWriter.writeStartElement("agent");
		xmlWriter.writeAttribute("x", QString::number(agent->getx()));
		xmlWriter.writeAttribute("y", QString::number(agent->gety()));
		//HACK: we have to write every agent on its own
		//      it's not directly possible to regenerate the groups of a source file
		xmlWriter.writeAttribute("n", QString::number(1));
		xmlWriter.writeAttribute("dx", QString::number(0));
		xmlWriter.writeAttribute("dy", QString::number(0));
		xmlWriter.writeAttribute("type", QString::number(agent->gettype()));

		// write addwaypoint
		foreach(Waypoint* waypoint, agent->waypoints) {
			xmlWriter.writeStartElement("addwaypoint");
			xmlWriter.writeAttribute("id", waypoint->id);
			xmlWriter.writeEndElement();
		}
		
		xmlWriter.writeEndElement();
	}

	// end writing file
	xmlWriter.writeEndDocument();

	// report success
	return true;
}

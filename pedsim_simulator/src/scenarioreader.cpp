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

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/scenarioreader.h>

#include <QFile>
#include <iostream>

#include <ros/ros.h>

ScenarioReader::ScenarioReader() {
  // initialize values
  currentAgents = nullptr;
  currentSpawnArea = nullptr;
}

bool ScenarioReader::readFromFile(const QString& filename) {
  ROS_DEBUG("Loading scenario file '%s'.", filename.toStdString().c_str());

  // open file
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    ROS_DEBUG("Couldn't open scenario file!");
    return false;
  }

  // read input
  xmlReader.setDevice(&file);

  while (!xmlReader.atEnd()) {
    xmlReader.readNext();
    processData();
  }

  // check for errors
  if (xmlReader.hasError()) {
    // TODO - fix qstring and std string issues here to show error lines
    ROS_DEBUG("Error while reading scenario file");
    return false;
  }

  // report success
  return true;
}

void ScenarioReader::processData() {
  if (xmlReader.isStartElement()) {
    const QString elementName = xmlReader.name().toString();
    const QXmlStreamAttributes elementAttributes = xmlReader.attributes();

    if ((elementName == "scenario") || (elementName == "welcome")) {
      // nothing to do
    } else if (elementName == "obstacle") {
      const double x1 = elementAttributes.value("x1").toString().toDouble();
      const double y1 = elementAttributes.value("y1").toString().toDouble();
      const double x2 = elementAttributes.value("x2").toString().toDouble();
      const double y2 = elementAttributes.value("y2").toString().toDouble();
      Obstacle* obs = new Obstacle(x1, y1, x2, y2);
      SCENE.addObstacle(obs);
    } else if (elementName == "waypoint") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double r = elementAttributes.value("r").toString().toDouble();
      // TODO - make the setting of waypoint behavior optional, 
      // and default to SIMPLE.
      const int b = elementAttributes.value("b").toString().toInt();
      AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
      w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(b));
      SCENE.addWaypoint(w);
    } else if (elementName == "queue") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double directionValue =
          elementAttributes.value("direction").toString().toDouble();

      const Ped::Tvector position(x, y);
      const Ped::Tangle direction = Ped::Tangle::fromDegree(directionValue);

      WaitingQueue* queue = new WaitingQueue(id, position, direction);
      SCENE.addWaitingQueue(queue);
    } else if (elementName == "attraction") {
      const QString id = elementAttributes.value("id").toString();
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const double width =
          elementAttributes.value("width").toString().toDouble();
      const double height =
          elementAttributes.value("height").toString().toDouble();
      const double strength =
          elementAttributes.value("strength").toString().toDouble();

      AttractionArea* attraction = new AttractionArea(id);
      attraction->setPosition(x, y);
      attraction->setSize(width, height);
      attraction->setStrength(strength);
      SCENE.addAttraction(attraction);
    } else if (elementName == "agent") {
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const int n = elementAttributes.value("n").toString().toInt();
      const double dx = elementAttributes.value("dx").toString().toDouble();
      const double dy = elementAttributes.value("dy").toString().toDouble();
      const int type = elementAttributes.value("type").toString().toInt();
      AgentCluster* agentCluster = new AgentCluster(x, y, n);
      agentCluster->setDistribution(dx, dy);

      /// TODO - change agents Vmax distribution based on agent type
      /// and other force parameters to realize different behaviours
      agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
      SCENE.addAgentCluster(agentCluster);
      currentAgents = agentCluster;
    } else if (elementName == "source") {
      const double x = elementAttributes.value("x").toString().toDouble();
      const double y = elementAttributes.value("y").toString().toDouble();
      const int n = elementAttributes.value("n").toString().toInt();
      const double dx = elementAttributes.value("dx").toString().toDouble();
      const double dy = elementAttributes.value("dy").toString().toDouble();
      const int type = elementAttributes.value("type").toString().toInt();
      AgentCluster* agentCluster = new AgentCluster(x, y, n);
      agentCluster->setDistribution(dx, dy);
      agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
      SCENE.addAgentCluster(agentCluster);
      currentAgents = agentCluster;

      SpawnArea* spawn_area = new SpawnArea(x, y, n, dx, dy);
      SCENE.addSpawnArea(spawn_area);
      currentSpawnArea = spawn_area;
    } else if (elementName == "addwaypoint") {
      if (currentAgents == nullptr) {
        ROS_DEBUG("Invalid <addwaypoint> element outside of agent element!");
        return;
      }

      // add waypoints to current <agent> element
      QString id = elementAttributes.value("id").toString();
      currentAgents->addWaypoint(SCENE.getWaypointByName(id));

      if (currentSpawnArea != nullptr) {
        currentSpawnArea->waypoints.emplace_back(id);
      }
    } else if (elementName == "addqueue") {
      if (currentAgents == nullptr) {
        ROS_DEBUG("Invalid <addqueue> element outside of agent element!");
        return;
      }

      // add waiting queue to current <agent> element
      QString id = elementAttributes.value("id").toString();
      currentAgents->addWaitingQueue(SCENE.getWaitingQueueByName(id));
    } else {
      // inform the user about invalid elements
      ROS_DEBUG("Unknown element: <%s>", elementName.toStdString().c_str());
    }
  } else if (xmlReader.isEndElement()) {
    const QString elementName = xmlReader.name().toString();

    if (elementName == "agent") {
      currentAgents = nullptr;
    }
  }
}

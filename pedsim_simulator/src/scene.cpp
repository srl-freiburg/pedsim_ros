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

#include <pedsim_simulator/config.h>
#include <pedsim_simulator/scene.h>

#include <pedsim/ped_tree.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/attractionarea.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/force/alongwallforce.h>
#include <pedsim_simulator/force/groupcoherenceforce.h>
#include <pedsim_simulator/force/groupgazeforce.h>
#include <pedsim_simulator/force/grouprepulsionforce.h>
#include <pedsim_simulator/force/randomforce.h>
#include <QGraphicsScene>

#include <ros/ros.h>

// initialize static value
Scene* Scene::Scene::instance = nullptr;

Scene::Scene(QObject* parent) {
  // initialize values
  sceneTime = 0;

  // TODO: create this dynamically according to scenario
  QRect area(-500, -500, 1000, 1000);

  // we need to add a tree to the scene to be able to search for neighbours
  tree =
      new Ped::Ttree(this, 0, area.x(), area.y(), area.width(), area.height());

  obstacle_cells_.clear();
}

Scene::~Scene() {
  // clean up
  clear();
}

Scene& Scene::getInstance() {
  // create an instance if there hasn't been one yet
  if (instance == nullptr) instance = new Scene();
  return *instance;
}

void Scene::clear() {
  // remove all elements from the scene
  Ped::Tscene::clear();

  // remove all agents
  // note: we don't need to delete them, because Ped::Tscene did so already
  agents.clear();

  // remove all waypoints
  // note: we don't need to delete them, because Ped::Tscene did so already
  waypoints.clear();

  // remove all obstacles
  // note: we don't need to delete them, because Ped::Tscene did so already
  obstacles.clear();

  // remove all agents groups
  foreach (AttractionArea* attraction, attractions)
    delete attraction;
  attractions.clear();

  // remove all agents clusters
  foreach (AgentCluster* agentCluster, agentClusters)
    delete agentCluster;
  agentClusters.clear();

  // remove all agents groups
  foreach (AgentGroup* group, agentGroups)
    delete group;
  agentGroups.clear();

  // don't clear the grid, because we can reuse it

  // reset time
  sceneTime = 0;
  emit sceneTimeChanged(sceneTime);
}

QRectF Scene::itemsBoundingRect() const {
  QRectF boundingRect(QPointF(-50, -50), QSizeF(100, 100));

  // iterate over all elements
  // → agents
  foreach (Agent* agent, agents) {
    if (!boundingRect.contains(agent->getVisiblePosition())) {
      // resize rectangle to include point
      boundingRect |=
          QRectF(agent->getVisiblePosition() - QPointF(0.5, 0.5), QSizeF(1, 1));
    }
  }
  // → obstacles
  foreach (Obstacle* obstacle, obstacles) {
    QPointF startPoint = obstacle->getVisiblePosition();
    QPointF endPoint(obstacle->getbx(), obstacle->getby());

    if (!boundingRect.contains(startPoint) ||
        !boundingRect.contains(endPoint)) {
      // resize rectangle to include point
      boundingRect |= QRectF(startPoint, endPoint);
    }
  }
  // → waypoints
  foreach (Waypoint* waypoint, waypoints) {
    AreaWaypoint* areaWaypoint = dynamic_cast<AreaWaypoint*>(waypoint);
    WaitingQueue* waitingQueue = dynamic_cast<WaitingQueue*>(waypoint);

    // → area waypoints
    if (areaWaypoint != nullptr) {
      if (!boundingRect.contains(areaWaypoint->getVisiblePosition())) {
        // resize rectangle to include point
        boundingRect |= QRectF(
            areaWaypoint->getVisiblePosition(),
            QSizeF(areaWaypoint->getRadius(), areaWaypoint->getRadius()));
      }
    }
    // → waiting queues
    else if (waitingQueue != nullptr) {
      if (!boundingRect.contains(waitingQueue->getVisiblePosition())) {
        // resize rectangle to include point
        boundingRect |=
            QRectF(waitingQueue->getVisiblePosition() - QPointF(0.5, 0.5),
                   QSizeF(1, 1));
      }
    }
  }
  // → agent clusters
  foreach (AgentCluster* agentCluster, agentClusters) {
    if (!boundingRect.contains(agentCluster->getVisiblePosition())) {
      // resize rectangle to include point
      boundingRect |= QRectF(
          agentCluster->getVisiblePosition() - QPointF(0.5, 0.5), QSizeF(1, 1));
    }
  }

  return boundingRect;
}

const QList<Agent*>& Scene::getAgents() const { return agents; }

QList<AgentGroup*> Scene::getGroups() { return agentGroups; }

QMap<QString, AttractionArea*> Scene::getAttractions() { return attractions; }

Agent* Scene::getAgentById(int idIn) const {
  foreach (Agent* currentAgent, agents) {
    if (idIn == currentAgent->getId()) return currentAgent;
  }

  return nullptr;
}

const QList<Obstacle*>& Scene::getObstacles() const { return obstacles; }

const QMap<QString, Waypoint*>& Scene::getWaypoints() const {
  return waypoints;
}

const QMap<QString, AttractionArea*>& Scene::getAttractions() const {
  return attractions;
}

Waypoint* Scene::getWaypointById(int idIn) const {
  foreach (Waypoint* currentWaypoint, waypoints) {
    if (idIn == currentWaypoint->getId()) return currentWaypoint;
  }

  return nullptr;
}

Waypoint* Scene::getWaypointByName(const QString& nameIn) const {
  return waypoints.value(nameIn);
}

WaitingQueue* Scene::getWaitingQueueByName(const QString& nameIn) const {
  Waypoint* waypoint = waypoints.value(nameIn);
  return dynamic_cast<WaitingQueue*>(waypoint);
}

const QList<AgentCluster*>& Scene::getAgentClusters() const {
  return agentClusters;
}

AttractionArea* Scene::getAttractionByName(const QString& nameIn) const {
  return attractions.value(nameIn);
}

AttractionArea* Scene::getClosestAttraction(const Ped::Tvector& positionIn,
                                            double* distanceOut) const {
  double minDistance = INFINITY;
  AttractionArea* minArg = nullptr;

  // find the attraction with minimal distance
  foreach (AttractionArea* attraction, attractions) {
    double distance = (attraction->getPosition() - positionIn).length();
    if (distance < minDistance) {
      minDistance = distance;
      minArg = attraction;
    }
  }

  // additionally return distance
  if (distanceOut != nullptr) *distanceOut = minDistance;

  return minArg;
}

double Scene::getTime() const { return sceneTime; }

bool Scene::hasStarted() const { return (sceneTime == 0); }

void Scene::dissolveClusters() {
  foreach (AgentCluster* cluster, agentClusters) {
    QList<Agent*> newAgents = cluster->dissolve();

    // divide agents into groups
    QList<AgentGroup*> newGroups = AgentGroup::divideAgents(newAgents);

    // apply group forces
    foreach (AgentGroup* currentGroup, newGroups) {
      if (currentGroup->memberCount() == 1) {
        // we don't need one agent groups
        delete currentGroup;
      } else if (currentGroup->memberCount() > 1) {
        // keep track of groups
        agentGroups.append(currentGroup);
      }

      // add group's agents to the scene
      foreach (Agent* currentAgent, currentGroup->getMembers()) {
        currentAgent->setWaypoints(cluster->getWaypoints());

        if (currentGroup->memberCount() > 1) {
          currentAgent->setGroup(currentGroup);
          // → Gaze Force
          GroupGazeForce* gazeForce = new GroupGazeForce(currentAgent);
          gazeForce->setGroup(currentGroup);
          currentAgent->addForce(gazeForce);

          // → Coherence Force
          GroupCoherenceForce* coherenceForce =
              new GroupCoherenceForce(currentAgent);
          coherenceForce->setGroup(currentGroup);
          currentAgent->addForce(coherenceForce);

          // → Repulsion Force
          GroupRepulsionForce* repulsionForce =
              new GroupRepulsionForce(currentAgent);
          repulsionForce->setGroup(currentGroup);
          currentAgent->addForce(repulsionForce);
        }
      }
    }

    // finally remove cluster
    delete cluster;
  }

  agentClusters.clear();
}

void Scene::addAgent(Agent* agent) {
  // keep track of the agent
  agents.append(agent);

  // add the agent to the PedSim scene
  Ped::Tscene::addAgent(agent);

  // add additional forces
  // → Random Force
  RandomForce* randomForce = new RandomForce(agent);
  agent->addForce(randomForce);
  // → Along Wall Force
  AlongWallForce* alongWallForce = new AlongWallForce(agent);
  agent->addForce(alongWallForce);

  // inform users
  emit agentAdded(agent->getId());
}

void Scene::addObstacle(Obstacle* obstacle) {
  // keep track of the obstacle
  obstacles.append(obstacle);

  // add the obstacle to the PedSim scene
  Ped::Tscene::addObstacle(obstacle);

  // inform users
  emit obstacleAdded(obstacle->getid());
}

void Scene::addWaypoint(Waypoint* waypoint) {
  // keep track of the waypoints
  waypoints.insert(waypoint->getName(), waypoint);

  // add the obstacle to the PedSim scene
  Ped::Tscene::addWaypoint(waypoint);

  // inform users
  emit waypointAdded(waypoint->getId());
}

void Scene::addAgentCluster(AgentCluster* clusterIn) {
  // keep track of agent clusters
  agentClusters.append(clusterIn);

  // inform users
  emit agentClusterAdded(clusterIn->getId());
}

void Scene::addWaitingQueue(WaitingQueue* queueIn) {
  // sanity checks
  if (queueIn == nullptr) {
    ROS_DEBUG("Cannot add null to the list of waiting queues!");
    return;
  }

  // add waiting queue as waypoint to the scene
  addWaypoint(dynamic_cast<Waypoint*>(queueIn));

  // inform users
  emit waitingQueueAdded(queueIn->getName());
}

void Scene::addAttraction(AttractionArea* attractionIn) {
  // sanity checks
  if (attractionIn == nullptr) {
    ROS_DEBUG("Cannot add null to the list of attractions!");
    return;
  }

  // add attraction to the scene
  attractions.insert(attractionIn->getName(), attractionIn);

  // inform users
  emit attractionAdded(attractionIn->getName());
}

bool Scene::removeAgent(Agent* agent) {
  // don't keep track of agent anymore
  agents.removeAll(agent);

  // remove agent from all groups
  QList<AgentGroup*> groupsToRemove;
  foreach (AgentGroup* currentGroup, agentGroups) {
    currentGroup->removeMember(agent);

    // check whether the group is empty and can be removed
    if (currentGroup->isEmpty()) groupsToRemove.append(currentGroup);
  }

  // remove unnecessary groups
  // note: use QObject::deleteLater() to keep the group valid till after the
  // agent's destructor
  foreach (AgentGroup* currentGroup, groupsToRemove) {
    agentGroups.removeAll(currentGroup);
    currentGroup->deleteLater();
  }

  // inform users
  emit agentRemoved(agent->getId());

  // actually remove it
  return Ped::Tscene::removeAgent(agent);
}

bool Scene::removeObstacle(Obstacle* obstacle) {
  // don't keep track of obstacle anymore
  obstacles.removeAll(obstacle);

  // inform users
  emit obstacleRemoved(obstacle->getid());

  // actually remove it
  return Ped::Tscene::removeObstacle(obstacle);
}

bool Scene::removeWaypoint(Waypoint* waypoint) {
  // don't keep track of waypoint anymore
  waypoints.remove(waypoint->getName());

  // remove waypoint from all agent clusters
  // (it is also removed from all agents in Ped::Tscene::removeWaypoint())
  foreach (AgentCluster* cluster, agentClusters)
    cluster->removeWaypoint(waypoint);

  // inform users
  emit waypointRemoved(waypoint->getId());

  // actually remove it
  return Ped::Tscene::removeWaypoint(waypoint);
}

bool Scene::removeAgentCluster(AgentCluster* clusterIn) {
  // don't keep track of agent cluster anymore
  int removedClusters = agentClusters.removeAll(clusterIn);

  // report when the cluster wasn't part of the scene
  if (removedClusters == 0) return false;

  // inform users
  emit agentClusterRemoved(clusterIn->getId());

  // free memory
  delete clusterIn;

  // report successful removal
  return true;
}

bool Scene::removeWaitingQueue(WaitingQueue* queueIn) {
  // sanity checks
  if (queueIn == nullptr) {
    ROS_DEBUG("Cannot remove null from the list of waiting queues!");
    return false;
  }

  // don't keep track of waiting queue anymore
  int removedCount = waypoints.remove(queueIn->getName());

  // check whether the queue was removed
  if (removedCount == 0) return false;

  // inform users
  emit waitingQueueRemoved(queueIn->getName());

  // delete object
  delete queueIn;

  // report successful removal
  return true;
}

bool Scene::removeAttraction(AttractionArea* attractionInIn) {
  // sanity checks
  if (attractionInIn == nullptr) {
    ROS_DEBUG("Cannot remove null from the list of attractions!");
    return false;
  }

  // don't keep track of attraction anymore
  int removedCount = attractions.remove(attractionInIn->getName());

  // check whether the queue was removed
  if (removedCount == 0) return false;

  // inform users
  emit attractionRemoved(attractionInIn->getName());

  // delete object
  delete attractionInIn;

  // report successful removal
  return true;
}

std::set<const Ped::Tagent*> Scene::getNeighbors(double x, double y,
                                                 double maxDist) {
  std::set<const Ped::Tagent*> potentialNeighbours =
      Ped::Tscene::getNeighbors(x, y, maxDist);
  Ped::Tvector position(x, y);

  // filter according to euclidean distance
  auto agentIter = potentialNeighbours.begin();
  while (agentIter != potentialNeighbours.end()) {
    const Ped::Tagent& candidate = **agentIter;
    Ped::Tvector candidatePos = candidate.getPosition();
    double distance = (candidatePos - position).length();

    // remove distant neighbors
    if (distance > maxDist) {
      potentialNeighbours.erase(agentIter++);
    } else {
      ++agentIter;
    }
  }

  return potentialNeighbours;
}

void Scene::moveAllAgents() {
  // inform users when there is going to be the first update
  if (sceneTime == 0) emit aboutToStart();

  // clean up scene if necessary
  double cleanupInterval = 2.0;
  if (fmod(sceneTime, cleanupInterval) < CONFIG.getTimeStepSize())
    cleanupScene();

  // inform users that there will be an update
  emit aboutToMoveAgents();

  // dissolve agent clusters
  if (!agentClusters.isEmpty()) dissolveClusters();

  // update scene time
  sceneTime += CONFIG.getTimeStepSize();
  emit sceneTimeChanged(sceneTime);

  // move the agents
  Ped::Tscene::moveAgents(CONFIG.getTimeStepSize());

  auto Dist = [](const double ax, const double ay, const double bx,
                 const double by) -> double {
    return std::hypot(ax - bx, ay - by);
  };

  // For every agents, if next WP is sink and 'close', call removeAgent.
  for (auto agent : getAgents()) {
    const auto agent_next_wp = agent->getCurrentWaypoint();
    // skip agents without any waypoint
    if (!agent_next_wp) {
      continue;
    }

    if (agent_next_wp->getBehavior() != Ped::Twaypoint::Behavior::SINK) {
      continue;
    }

    const double d = Dist(agent_next_wp->getx(), agent_next_wp->gety(),
                          agent->getx(), agent->gety());
    if (d < agent_next_wp->getRadius()) {
      // At sink waypoint.
      ROS_DEBUG_STREAM("Killing agent: " << agent->getId());
      removeAgent(agent);
    }
  }

  // inform users
  emit movedAgents();
}

void Scene::cleanupScene() { Ped::Tscene::cleanup(); }

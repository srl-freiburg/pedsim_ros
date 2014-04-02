// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2003 - 2012 by Christian Gloor

#ifndef _scene_h_
#define _scene_h_

// Includes
// → PedSim
#include <libpedsim/ped_scene.h>
#include <libpedsim/ped_vector.h>
// → Qt
#include <QMap>
#include <QRectF>
#include <QObject>

#include <pedsim_simulator/utilities.h>


// Forward Declarations
class QGraphicsScene;
class Agent;
class Obstacle;
class Waypoint;
class AttractionArea;
class AgentCluster;
class AgentGroup;
class WaitingQueue;


class Scene : public QObject, protected Ped::Tscene {
	Q_OBJECT

public:
	Scene(QObject* parent = 0);
	virtual ~Scene();


    // Singleton Design Pattern
	#define SCENE Scene::getInstance()
protected:
	static Scene* instance;
public:
	static Scene& getInstance();


	// Signals
signals:
	void aboutToStart();
	void aboutToMoveAgents();
	void movedAgents();
	void sceneTimeChanged(double time);

	// → added/removed elements
	void agentAdded(int id);
	void agentRemoved(int id);
	void obstacleAdded(int id);
	void obstacleRemoved(int id);
	void waypointAdded(int id);
	void waypointRemoved(int id);
	void agentClusterAdded(int id);
	void agentClusterRemoved(int id);
	void waitingQueueAdded(QString name);
	void waitingQueueRemoved(QString name);
	void attractionAdded(QString name);
	void attractionRemoved(QString name);


	// Slots
public slots:
	void onSimulationSpeedChanged(int simSpeed);
	void moveAllAgents();
protected slots:
	void cleanupScene();


	// Methods
public:
	void clear();
	void drawObstacles ( float x1, float y1, float x2, float y2 );

	QRectF itemsBoundingRect() const;

	// → elements
	const QList<Agent*>& getAgents() const;
	Agent* getAgentById(int idIn) const;
	QList<AgentGroup*> getGroups();
	const QList<Obstacle*>& getObstacles() const;
	const QMap<QString, Waypoint*>& getWaypoints() const;
	const QMap<QString, AttractionArea*>& getAttractions() const;
	Waypoint* getWaypointById(int idIn) const;
	Waypoint* getWaypointByName(const QString& nameIn) const;
	WaitingQueue* getWaitingQueueByName(const QString& nameIn) const;
	const QList<AgentCluster*>& getAgentClusters() const;
	AttractionArea* getAttractionByName(const QString& nameIn) const;
	AttractionArea* getClosestAttraction(const Ped::Tvector& positionIn, double* distanceOut = nullptr) const;

	// → simulation time
	double getTime() const;
	bool hasStarted() const;

	// → Cluster→Agents
protected:
	void dissolveClusters();

	// → libPedSim overrides
public:
	virtual void addAgent(Agent* agent);
	virtual void addObstacle(Obstacle* obstacle);
	virtual void addWaypoint(Waypoint* waypoint);
	virtual void addAgentCluster(AgentCluster* clusterIn);
	virtual void addWaitingQueue(WaitingQueue* queueIn);
	virtual void addAttraction(AttractionArea* attractionIn);
	virtual bool removeAgent(Agent* agent);
	virtual bool removeObstacle(Obstacle* obstacle);
	virtual bool removeWaypoint(Waypoint* waypoint);
	virtual bool removeAgentCluster(AgentCluster* clusterIn);
	virtual bool removeWaitingQueue(WaitingQueue* queueIn);
	virtual bool removeAttraction(AttractionArea* attractionInIn);

	virtual std::set<const Ped::Tagent*> getNeighbors(double x, double y, double maxDist);

	// obstacle cell locations
    std::vector<Location> obstacle_cells_;

	// Attributes
protected:
	QList<Agent*> agents;
	QList<Obstacle*> obstacles;
	QMap<QString, Waypoint*> waypoints;
	QMap<QString, AttractionArea*> attractions;
	QList<AgentCluster*> agentClusters;
	QList<AgentGroup*> agentGroups;

	// → simulated time
	double sceneTime;


};

#endif

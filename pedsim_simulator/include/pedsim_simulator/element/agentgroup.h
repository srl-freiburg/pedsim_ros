// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _agentgroup_h_
#define _agentgroup_h_

// Includes
// → SGDiCoP
#include "scenarioelement.h"
// → PedSim
#include <libpedsim/ped_vector.h>
// → Qt
#include <QGraphicsItemGroup>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QTimer>


// Forward Declarations
class Agent;
// class AgentGroupRepresentation;


class AgentGroup : public ScenarioElement {
	Q_OBJECT
	
	// Constructor and Destructor
public:
	AgentGroup();
	AgentGroup(const QList<Agent*>& agentsIn);
	AgentGroup(std::initializer_list<Agent*>& agentsIn);
	virtual ~AgentGroup();


	// Signals
signals:
	void centerOfMassChanged(double x, double y);
	void membersChanged();
	void memberAdded(int id);
	void memberRemoved(int id);


	// Slots
public slots:
	void onPositionChanged(double x, double y);


	// Static Methods
public:
	static QList<AgentGroup*> divideAgents(const QList<Agent*>& agentsIn);
	// → Helper
protected:
	static void reportSizeDistribution(const QVector<int>& sizeDistributionIn);


	// Methods
	// → Group members
public:
	QList<Agent*>& getMembers();
	const QList<Agent*>& getMembers() const;
	bool addMember(Agent* agentIn);
	bool removeMember(Agent* agentIn);
	bool setMembers(const QList<Agent*>& agentsIn);
	bool isEmpty() const;
	int memberCount() const;
	
	// → Center of Mass
public:
// 	QPointF getCenterOfMass() const;
	Ped::Tvector getCenterOfMass() const;
protected slots:
	Ped::Tvector updateCenterOfMass();

	// → Recollection
public:
	void setRecollect(bool recollectIn);
	bool isRecollecting() const;
	double getMaxDistance();
protected:
	void updateMaxDistance();

	// → ScenarioElement Overrides
public:
	virtual QString toString() const;


	// Attributes
protected:
	QList<Agent*> members;

	// → Center of Mass
	bool dirty;
	Ped::Tvector cacheCoM;
	//   → delayed update
	QTimer comUpdateTimer;

	// → recollecting group
	bool recollecting;
	bool dirtyMaxDistance;
	double cacheMaxDistance;

	// → graphical representation
// 	AgentGroupRepresentation* representation;
};

#endif

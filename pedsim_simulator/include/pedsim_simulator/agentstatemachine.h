// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

#ifndef _agentstatemachine_h_
#define _agentstatemachine_h_

// Includes
// → Qt
#include <QObject>

// Forward Declarations
class Agent;
class AttractionArea;
class IndividualWaypointPlanner;
class QueueingWaypointPlanner;
class GroupWaypointPlanner;
class ShoppingPlanner;


class AgentStateMachine : public QObject {
	Q_OBJECT

	// Enums
public:
	typedef enum {
		StateNone,
		StateWaiting,
		StateQueueing,
		StateWalking,
		StateGroupWalking,
		StateShopping
	} AgentState;
	Q_ENUMS(AgentState)


	// Constructor and Destructor
public:
	AgentStateMachine(Agent* agentIn);
	virtual ~AgentStateMachine();


	// Signals
signals:
	void stateChanged(AgentState newState);


public slots:
	void loseAttraction();


	// Methods
public:
	void doStateTransition();
	AgentState getCurrentState();
protected:
	void activateState(AgentState stateIn);
	void deactivateState(AgentState stateIn);
	bool checkGroupForAttractions(AttractionArea** attractionOut = nullptr) const;
	QString stateToName(AgentState stateIn) const;


	// Attributes
protected:
	Agent* agent;

	// → State Machine
	AgentState state;
	AgentState normalState;

	// → Waypoint Planner
	IndividualWaypointPlanner* individualPlanner;
	QueueingWaypointPlanner* queueingPlanner;
	GroupWaypointPlanner* groupWaypointPlanner;
	ShoppingPlanner* shoppingPlanner;

	// → Attraction
	AttractionArea* groupAttraction;
	bool shallLoseAttraction;
};

#endif

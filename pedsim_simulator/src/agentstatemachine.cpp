// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/agentstatemachine.h>

// → SGDiCoP
// #include "logging.h"
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/waitingqueue.h>
#include <pedsim_simulator/element/attractionarea.h>

#include <pedsim_simulator/waypointplanner/individualwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/queueingplanner.h>
#include <pedsim_simulator/waypointplanner/groupwaypointplanner.h>
#include <pedsim_simulator/waypointplanner/shoppingplanner.h>
// → Qt
#include <QSettings>


AgentStateMachine::AgentStateMachine(Agent* agentIn) {
	// initialize values
	agent = agentIn;
	individualPlanner = nullptr;
	queueingPlanner = nullptr;
	groupWaypointPlanner = nullptr;
	shoppingPlanner = nullptr;
	groupAttraction = nullptr;
	shallLoseAttraction = false;

	// initialize state machine
	state = StateNone;
}

AgentStateMachine::~AgentStateMachine() {
	// clean up
	delete individualPlanner;
	delete queueingPlanner;
	delete groupWaypointPlanner;
	delete shoppingPlanner;
}

void AgentStateMachine::loseAttraction() {
	// set mark to lose attraction
	shallLoseAttraction = true;
}

void AgentStateMachine::doStateTransition() {
	// determine new state
	// → randomly get attracted by attractions
	if((state != StateShopping) && (state != StateQueueing)) {
		double distance = INFINITY;
		AttractionArea* attraction = nullptr;
		bool hasGroupAttraction = checkGroupForAttractions(&attraction);
		if(hasGroupAttraction) {
			// inherit groups' attraction
			groupAttraction = attraction;

			normalState = state;
			activateState(StateShopping);
			return;
		}
		else {
			//TODO: attraction must be visible!
			attraction = SCENE.getClosestAttraction(agent->getPosition(), &distance);

			if(attraction != nullptr) {
				// check whether agent is attracted
				// Note: The Cumulative Geometric Distribution determines the
				//       number of Bernoulli trials needed to get one success.
				//       → CDF(X) = 1-(1-p)^k   with k = the number of trials
				QSettings settings;
				double baseProbability = settings.value("AgentStateMachine/AttractionProbability", 0.02).toDouble();
				double maxAttractionDistance = settings.value("AgentStateMachine/MaxAttractionDistance", 7).toDouble();
				// → probability dependents on strength, distance,
				//   and whether another group member are attracted
				double probability = baseProbability 
					* attraction->getStrength()
					* ((distance<maxAttractionDistance)?(1-(distance/maxAttractionDistance)):0)
					* CONFIG.timeStepSize;
				std::bernoulli_distribution isAttracted(probability);

				if(isAttracted(RNG())) {
					normalState = state;
					activateState(StateShopping);
					return;
				}
			}
		}
	}
	// → randomly lose attraction
	if(state == StateShopping) {
		// check whether agent loses attraction
		//TODO: make this dependent from the distance to CoM
		QSettings settings;
		double probability = settings.value("AgentStateMachine/AttractionLoseProbability", 0.03).toDouble();
		std::bernoulli_distribution isAttracted(probability * CONFIG.timeStepSize);
		
		if(shallLoseAttraction || isAttracted(RNG())) {
			// reactivate previous state
			activateState(normalState);

			// alreade picked a new state, so nothing to do
			return;
		}
	}

	// → operate on waypoints/destinations
	if((state == StateNone) || agent->needNewDestination()) {
		Ped::Twaypoint* destination = agent->updateDestination();
		Waypoint* waypoint = dynamic_cast<Waypoint*>(destination);
		//TODO: move this to the agent
		WaitingQueue* waitingQueue = dynamic_cast<WaitingQueue*>(waypoint);
		
		if(destination == nullptr)
			activateState(StateWaiting);
		else if(waitingQueue != nullptr)
			activateState(StateQueueing);
		else {
			if(agent->isInGroup())
				activateState(StateGroupWalking);
			else 
				activateState(StateWalking);
		}
	}
}

void AgentStateMachine::activateState(AgentState stateIn) {
// 	DEBUG_LOG("Agent %1 activating state '%2' (time: %3)",
// 		agent->getId(), stateToName(stateIn),
// 		SCENE.getTime());

	// de-activate old state
	deactivateState(state);

	// re-activate all forces
	agent->enableAllForces();

	// set state
	state = stateIn;

	Waypoint* destination = dynamic_cast<Waypoint*>(agent->getCurrentDestination());

	switch(state) {
		case StateNone:
			agent->setWaypointPlanner(nullptr);
			break;
		case StateWaiting:
			agent->setWaypointPlanner(nullptr);
			break;
		case StateWalking:
			if(individualPlanner == nullptr)
				individualPlanner = new IndividualWaypointPlanner();
			individualPlanner->setAgent(agent);
			individualPlanner->setDestination(destination);
			agent->setWaypointPlanner(individualPlanner);
			break;
		case StateQueueing:
			if(queueingPlanner == nullptr)
				queueingPlanner = new QueueingWaypointPlanner();
			queueingPlanner->setAgent(agent);
			queueingPlanner->setDestination(destination);
			agent->setWaypointPlanner(queueingPlanner);
			break;
		case StateGroupWalking:
			if(groupWaypointPlanner == nullptr)
				groupWaypointPlanner = new GroupWaypointPlanner();
			groupWaypointPlanner->setDestination(destination);
			groupWaypointPlanner->setGroup(agent->getGroup());
			agent->setWaypointPlanner(groupWaypointPlanner);
			break;
		case StateShopping:
			shallLoseAttraction = false;
			if(shoppingPlanner == nullptr)
				shoppingPlanner = new ShoppingPlanner();
			AttractionArea* attraction = SCENE.getClosestAttraction(agent->getPosition());
			shoppingPlanner->setAgent(agent);
			shoppingPlanner->setAttraction(attraction);
			agent->setWaypointPlanner(shoppingPlanner);
			agent->disableForce("GroupCoherence");
			agent->disableForce("GroupGaze");

			// keep other agents informed about the attraction
			AgentGroup* group = agent->getGroup();
			if(group != nullptr) {
				foreach(Agent* member, group->getMembers()) {
					if(member == agent)
						continue;

					AgentStateMachine* memberStateMachine = member->getStateMachine();
					connect(shoppingPlanner, SIGNAL(lostAttraction()),
						memberStateMachine, SLOT(loseAttraction()));
				}
			}

			break;
	}

	// inform users
	emit stateChanged(state);
}

void AgentStateMachine::deactivateState(AgentState state) {
	switch(state) {
		case StateNone:
			// nothing to do
			break;
		case StateWaiting:
			// nothing to do
			break;
		case StateWalking:
			// nothing to do
			break;
		case StateQueueing:
			// nothing to do
			break;
		case StateGroupWalking:
			// nothing to do
			break;
		case StateShopping:
			// inform other group members
			shoppingPlanner->loseAttraction();

			// don't worry about other group members
			AgentGroup* group = agent->getGroup();
			if(group != nullptr) {
				foreach(Agent* member, group->getMembers()) {
					if(member == agent)
						continue;

					AgentStateMachine* memberStateMachine = member->getStateMachine();
					disconnect(shoppingPlanner, SIGNAL(lostAttraction()),
						memberStateMachine, SLOT(loseAttraction()));
				}
			}

			break;
	}
}

bool AgentStateMachine::checkGroupForAttractions(AttractionArea** attractionOut) const {
	AgentGroup* group = agent->getGroup();

	// check whether the agent is even in a group
	if(group == nullptr) {
		if(attractionOut != nullptr)
			*attractionOut = nullptr;
		return false;
	}

	// check all group members
	foreach(Agent* member, group->getMembers()) {
		// ignore agent himself
		if(member == agent)
			continue;

		// check whether the group member uses ShoppingPlanner
		WaypointPlanner* planner = member->getWaypointPlanner();
		ShoppingPlanner* typedPlanner = dynamic_cast<ShoppingPlanner*>(planner);
		if(typedPlanner != nullptr) {
			AttractionArea* attraction = typedPlanner->getAttraction();

			if(attraction != nullptr) {
// 				DEBUG_LOG("Agent%1's Group Member (%2) is attracted to: %3",
// 					agent->getId(), member->getId(), attraction->getName());
				attractionOut = &attraction;
				return true;
			}
		}
	}

	// no group member is attracted to something
	if(attractionOut != nullptr)
		*attractionOut = nullptr;
	return false;
}

QString AgentStateMachine::stateToName(AgentState stateIn) const {
	switch(stateIn) {
		case StateNone:
			return "StateNone";
		case StateWaiting:
			return "StateWaiting";
		case StateWalking:
			return "StateWalking";
		case StateQueueing:
			return "StateQueueing";
		case StateGroupWalking:
			return "StateGroupWalking";
		case StateShopping:
			return "StateShopping";
		default:
			return "UnknownState";
	}
}

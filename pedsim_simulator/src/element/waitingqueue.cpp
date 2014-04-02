// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/element/waitingqueue.h>

// → SGDiCoP
// #include "logging.h"
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/element/agent.h>

// #include "visual/waitingqueuerepresentation.h"
// → Qt
#include <QSettings>


WaitingQueue::WaitingQueue(const QString& nameIn, Ped::Tvector positionIn, Ped::Tangle directionIn)
	: Waypoint(nameIn, positionIn), direction(directionIn) {
	// initialize values
	dequeueTime = INFINITY;
	waitDurationMean = 10;
	waitDurationStd = 5;

	// graphical representation
// 	representation = new WaitingQueueRepresentation(this);
	
	// connect signals
	connect(&SCENE, SIGNAL(sceneTimeChanged(double)), this, SLOT(onTimeChanged(double)));
}

WaitingQueue::~WaitingQueue() {
	// clean up
// 	delete representation;
}

void WaitingQueue::onTimeChanged(double timeIn) {
	// skip when there is none 
	if(queuedAgents.empty()) {
		return;
	}

	Agent* firstInLine = queuedAgents.first();
	
	// check whether waiting started
	if(std::isinf(dequeueTime)) {
		if(hasReachedWaitingPosition()) {
			// set the time when to dequeue leading agent
// 			DEBUG_LOG("Waiting Time for agent (%1) starts!", firstInLine->toString());
			startDequeueTime();
		}
	}
	
	// let first agent in line pass
	if(dequeueTime <= timeIn) {
		// dequeue agent and inform users
// 		DEBUG_LOG("Agent %1 may pass Queue (%2)", firstInLine->getId(), this->toString());
		emit agentMayPass(firstInLine->getId());
		dequeueAgent(firstInLine);
	}
}

void WaitingQueue::onLastAgentPositionChanged(double xIn, double yIn) {
	emit queueEndPositionChanged(xIn, yIn);
}

Ped::Tangle WaitingQueue::getDirection() const {
	return direction;
}

void WaitingQueue::setDirection(const Ped::Tangle& angleIn) {
	direction = angleIn;

	// inform users
	emit directionChanged(direction.toRadian());
}

void WaitingQueue::setDirection(double xIn, double yIn) {
	setDirection(Ped::Tvector(xIn, yIn));
}

void WaitingQueue::setDirection(const Ped::Tvector& directionIn) {
	direction = directionIn.polarAngle();

	// inform users
	emit directionChanged(direction.toRadian());
}

bool WaitingQueue::isEmpty() const {
	return queuedAgents.isEmpty();
}

Ped::Tvector WaitingQueue::getQueueEndPosition() const {
	if(queuedAgents.isEmpty())
		return position;
	else
		return queuedAgents.last()->getPosition();
}

const Agent* WaitingQueue::enqueueAgent(Agent* agentIn) {
	// determine output
	const Agent* aheadAgent = (queuedAgents.isEmpty())?nullptr:queuedAgents.last();

	// add agent to queue
	queuedAgents.append(agentIn);

	// inform about new first in line
	if(aheadAgent == nullptr) {
		emit queueLeaderChanged(agentIn->getId());
// 		DEBUG_LOG("New Queue Leader: %1 (queue: %2)", agentIn->getId(), this->toString());
	}

	// stay informed about updates on queue end
	connect(agentIn, SIGNAL(positionChanged(double,double)),
		this, SLOT(onLastAgentPositionChanged(double,double)));
	// ignore updates from previous queue end
	if(aheadAgent != nullptr) {
		disconnect(aheadAgent, SIGNAL(positionChanged(double,double)),
			this, SLOT(onLastAgentPositionChanged(double,double)));
	}

	// inform users
	emit queueEndChanged();
	informAboutEndPosition();

	// return agent ahead of the new agent
	return aheadAgent;
}

bool WaitingQueue::dequeueAgent(Agent* agentIn) {
	// sanity checks
	if(queuedAgents.isEmpty()) {
// 		ERROR_LOG("Cannot dequeue agent from empty waiting queue!");
		return false;
	}

	// remove agent from queue
	bool dequeueSuccess;
	bool dequeuedWasFirst = (queuedAgents.first() == agentIn);
	bool dequeuedWasLast = (queuedAgents.last() == agentIn);
	if(dequeuedWasFirst) {
		queuedAgents.removeFirst();
		dequeueSuccess = true;
	}
	else {
// 		WARN_LOG("Dequeueing agent from queue (%1), although it isn't at the front of the queue",
// 			agentIn->toString());
		int removedCount = queuedAgents.removeAll(agentIn);
		dequeueSuccess = (removedCount >= 1);
		
		if(dequeueSuccess == false) {
// 			ERROR_LOG("Agent isn't waiting in queue! (Agent: %1, Queue: %2)",
// 				agentIn->toString(), this->toString());
			return false;
		}
	}

	// inform other agents
	emit agentDequeued(agentIn->getId());

	// update leading position
	if(dequeuedWasFirst) {
		// determine new first agent in line
		const Agent* newFront = (queuedAgents.isEmpty())? nullptr : queuedAgents.first();
		
		// reset time for next agent
		resetDequeueTime();
		
		// inform users about changed front position
		int frontId = (newFront != nullptr)? newFront->getId() : -1;
		emit queueLeaderChanged(frontId);
	}

	// update queue end
	if(dequeuedWasLast) {
		disconnect(agentIn, SIGNAL(positionChanged(double,double)),
			this, SLOT(onLastAgentPositionChanged(double,double)));

		emit queueEndChanged();
		informAboutEndPosition();
	}

	return dequeueSuccess;
}

bool WaitingQueue::hasReachedWaitingPosition() {
	if(queuedAgents.isEmpty())
		return false;

	const double waitingRadius = 0.7;

	// compute distance from where queue starts
	const Agent* leadingAgent = queuedAgents.first();
	Ped::Tvector diff = leadingAgent->getPosition() - position;
	return (diff.length() < waitingRadius);
}

void WaitingQueue::resetDequeueTime() {
	dequeueTime = INFINITY;
}

void WaitingQueue::startDequeueTime() {
	// draw random waiting period
	normal_distribution<double> distribution(waitDurationMean, waitDurationStd);
	double waitDuration = distribution(RNG());
	dequeueTime = SCENE.getTime() + waitDuration;
}

void WaitingQueue::informAboutEndPosition() {
	// inform users
	if(queuedAgents.isEmpty()) {
		emit queueEndPositionChanged(position.x, position.y);
	}
	else {
		Agent* lastAgent = queuedAgents.last();
		Ped::Tvector endPosition = lastAgent->getPosition();
		emit queueEndPositionChanged(endPosition.x, endPosition.y);
	}
}

Ped::Tvector WaitingQueue::closestPoint(const Ped::Tvector& p, bool* withinWaypoint) const {
	return getQueueEndPosition();
}

QPointF WaitingQueue::getVisiblePosition() const {
	return QPointF(position.x, position.y);
}

void WaitingQueue::setVisiblePosition(const QPointF& positionIn) {
	setPosition(positionIn.x(), positionIn.y());
}

QString WaitingQueue::toString() const {
	QStringList waitingIDs;
	foreach(const Agent* agent, queuedAgents)
		waitingIDs.append(QString::number(agent->getId()));
	QString waitingString = waitingIDs.join(",");
	
	return tr("WaitingQueue '%1' (@%2,%3; queue: %4)")
		.arg(name)
		.arg(position.x).arg(position.y)
		.arg(waitingString);
}

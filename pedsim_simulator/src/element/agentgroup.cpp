// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
// Copyright (c) 2013 by Sven Wehner

// Includes
#include <pedsim_simulator/element/agentgroup.h>

// → SGDiCoP
// #include "logging.h"
#include <pedsim_simulator/rng.h>
#include <pedsim_simulator/element/agent.h>

// #include "visual/agentgrouprepresentation.h"
// → Qt
#include <QSettings>


AgentGroup::AgentGroup() {
	// initialize values
	dirty = true;
	dirtyMaxDistance = true;
	recollecting = true;
	// → delayed center of mass update
	comUpdateTimer.setSingleShot(true);
	comUpdateTimer.setInterval(0);
	connect(&comUpdateTimer, SIGNAL(timeout()), this, SLOT(updateCenterOfMass()));

	// compute center of mass
	updateCenterOfMass();

	// graphical representation
// 	representation = new AgentGroupRepresentation(this);
}

AgentGroup::AgentGroup(const QList<Agent*>& agentsIn) {
	// initialize values
	dirty = true;
	dirtyMaxDistance = true;
	members = agentsIn;
	// → delayed center of mass update
	comUpdateTimer.setSingleShot(true);
	comUpdateTimer.setInterval(0);
	connect(&comUpdateTimer, SIGNAL(timeout()), this, SLOT(updateCenterOfMass()));

	// compute center of mass
	updateCenterOfMass();

	// graphical representation
// 	representation = new AgentGroupRepresentation(this);

	// connect signals
	foreach(Agent* agent, members)
		connect(agent, SIGNAL(positionChanged(double,double)),
			this, SLOT(onPositionChanged(double,double)));
}

AgentGroup::AgentGroup(std::initializer_list<Agent*>& agentsIn) {
	// initialize values
	dirty = true;
	dirtyMaxDistance = true;
	comUpdateTimer.setSingleShot(true);
	comUpdateTimer.setInterval(0);
	connect(&comUpdateTimer, SIGNAL(timeout()), this, SLOT(updateCenterOfMass()));
	
	// add agents from initializer_list to the member list
	for(Agent* currentAgent : agentsIn) {
		members.append(currentAgent);
		connect(currentAgent, SIGNAL(positionChanged(double,double)),
			this, SLOT(onPositionChanged(double,double)));
	}

	// compute center of mass
	updateCenterOfMass();
	
	// graphical representation
// 	representation = new AgentGroupRepresentation(this);
}

AgentGroup::~AgentGroup() {
	// clean up
	// → remove graphical representation
// 	delete representation;
}

void AgentGroup::onPositionChanged(double x, double y) {
	// mark center of mass as dirty (needs to be re-calculated)
	dirty = true;
	dirtyMaxDistance = true;
	comUpdateTimer.start();
}

QList<AgentGroup*> AgentGroup::divideAgents(const QList<Agent*>& agentsIn) {
	QList<AgentGroup*> groups;
	QList<Agent*> unassignedAgents = agentsIn;
	
	// initialize Poisson distribution
	// → read settings
	QSettings settings;
	const double poissonLambda = settings.value("GroupSizeDistribution/Lambda", 1).toDouble();
	// → initialize distribution
	std::poisson_distribution<int> distribution(poissonLambda);

	// distribution of group sizes
	QVector<int> sizeDistribution;
	// → create distribution
	int agentCount = agentsIn.count();
	int sizeSum = 0;
	while(sizeSum < agentCount) {
		// randomly draw the group size (Poisson distribution)
		// (don't use group size = 0)
		int groupSize;
		do {
			groupSize = distribution(RNG());
		} while(groupSize == 0);
		// → limit group size to the number of agents left
		groupSize = min(groupSize, agentCount-sizeSum);

		// → record group size
		if(sizeDistribution.size() < groupSize)
			sizeDistribution.resize(groupSize);
		sizeDistribution[groupSize-1]++;

		// → update sum over all group sizes
		sizeSum += groupSize;
	}

	// → report group size distribution
	reportSizeDistribution(sizeDistribution);

	// → iterate over all group sizes and create groups accordingly
	//   (start with the largest size to receive contiguous groups)
	for(int groupSize = sizeDistribution.count(); groupSize > 0; --groupSize) {
		// create groups of given size
		for(int groupIter = 0; groupIter < sizeDistribution[groupSize-1]; ++groupIter) {
			// create a group
			AgentGroup* newGroup = new AgentGroup();
			// and add it to result set
			groups.append(newGroup);

			// add first agent to the group
			Agent* groupLeader = unassignedAgents.takeFirst();
			Ped::Tvector leaderPosition = groupLeader->getPosition();
			newGroup->addMember(groupLeader);
			
			// add other agents to group
			QList<QPair<Agent*,double> > distanceList;
			foreach(Agent* potentialMember, unassignedAgents) {
				Ped::Tvector position = potentialMember->getPosition();
				double distance = (leaderPosition - position).length();
				
				// add potential group member to the list according to the distance
				auto iter = distanceList.begin();
				while(iter < distanceList.end()) {
					if(distance > iter->second)
						break;
					else
						++iter;
				}
				// → insert candidate
				distanceList.insert(iter, qMakePair(potentialMember, distance));

				// reduce list if necessary
				if(distanceList.size() > groupSize-1)
					distanceList.removeFirst();
			}
			
			// add neighbors to the group
			foreach(const auto& member, distanceList) {
				newGroup->addMember(member.first);
				
				// don't consider the group member as part of another group
				unassignedAgents.removeOne(member.first);
			}
		}
	}

	return groups;
}

QList<Agent*>& AgentGroup::getMembers() {
	return members;
}

const QList<Agent*>& AgentGroup::getMembers() const {
	return members;
}

bool AgentGroup::addMember(Agent* agentIn) {
	if(members.contains(agentIn)) {
// 		DEBUG_LOG("AgentGroup: Couldn't add Agent twice!");
		return false;
	}

	// add Agent to the group and mark cache invalid
	members.append(agentIn);
	dirty = true;
	dirtyMaxDistance = true;
	comUpdateTimer.start();
	
	// connect signals
	connect(agentIn, SIGNAL(positionChanged(double,double)),
		this, SLOT(onPositionChanged(double,double)));

	// inform users
	emit memberAdded(agentIn->getId());

	return true;
}

bool AgentGroup::removeMember(Agent* agentIn) {
	bool hasRemovedMember = members.removeOne(agentIn);
	
	// mark cache invalid, if the agent has been removed
	if(hasRemovedMember == true) {
		// disconnect signals
		disconnect(agentIn, SIGNAL(positionChanged(double,double)),
			this, SLOT(onPositionChanged(double,double)));

		// invalidate cache and schedule update
		dirty = true;
		dirtyMaxDistance = true;
		comUpdateTimer.start();
		
		// inform users
		emit memberRemoved(agentIn->getId());

		return true;
	}
	else {
		return false;
	}
}

bool AgentGroup::setMembers(const QList<Agent*>& agentsIn) {
	// set the new members and mark cache invalid
	members = agentsIn;
	dirty = true;
	dirtyMaxDistance = true;
	comUpdateTimer.start();

	// connect signals
	foreach(Agent* agent, members)
		connect(agent, SIGNAL(positionChanged(double,double)),
			this, SLOT(onPositionChanged(double,double)));

	// inform users
	emit membersChanged();

	return true;
}

bool AgentGroup::isEmpty() const {
	return members.isEmpty();
}

int AgentGroup::memberCount() const {
	return members.count();
}

Ped::Tvector AgentGroup::getCenterOfMass() const {
	// check cache
	if(dirty) {
		// update cache
		AgentGroup* nonConstThis = const_cast<AgentGroup*>(this);
		nonConstThis->updateCenterOfMass();
	}
	
	return cacheCoM;
}

Ped::Tvector AgentGroup::updateCenterOfMass() {
	if(!dirty)
		return cacheCoM;
	
	// compute center of mass
	Ped::Tvector com;
	foreach(const Agent* member, members)
		com += member->getPosition();
	int groupSize = members.size();
	com /= groupSize;

	// set cache value
	cacheCoM = com;

	// mark the cache as valid
	dirty = false;

	// update graphical representation
	emit centerOfMassChanged(cacheCoM.x, cacheCoM.y);

	return cacheCoM;
}

void AgentGroup::setRecollect(bool recollectIn) {
	if(recollectIn) {
		// check whether recollecting mode has already been activated
		if(recollecting)
			return;

// 		DEBUG_LOG("AgentGroup needs to recollect! (%1)", toString());
		recollecting = true;
	}
	else {
		// check whether recollecting mode hasn't been activated
		if(!recollecting)
			return;

// 		DEBUG_LOG("AgentGroup finished recollecting! (%1)", toString());
		recollecting = false;
	}
}

bool AgentGroup::isRecollecting() const {
	return recollecting;
}

double AgentGroup::getMaxDistance() {
	if(dirty || dirtyMaxDistance)
		updateMaxDistance();

	return cacheMaxDistance;
}

void AgentGroup::updateMaxDistance() {
	Ped::Tvector com = getCenterOfMass();
	double maxDistance = 0;
	foreach(Agent* agent, members) {
		double distance = (com - agent->getPosition()).length();
		if(distance > maxDistance)
			maxDistance = distance;
	}
	cacheMaxDistance = maxDistance;
	dirtyMaxDistance = false;
}

void AgentGroup::reportSizeDistribution(const QVector<int>& sizeDistributionIn) {
	QString sizeDistributionString;
	int groupSize = 1;
	foreach(int count, sizeDistributionIn) {
		sizeDistributionString += tr(" %1: %2;").arg(groupSize).arg(count);
		groupSize++;
	}
// 	INFO_LOG("Group Size Distribution:%1", sizeDistributionString);
}

QString AgentGroup::toString() const {
	QString agentString;
	bool firstMember = true;
	foreach(Agent* agent, members) {
		if(!firstMember)
			agentString += ", ";
		agentString += agent->toString();
		firstMember = false;
	}
	
	return tr("AgentGroup (CoM: @%1,%2; Members:%3)").arg(cacheCoM.x).arg(cacheCoM.y).arg(agentString);
}

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
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/rng.h>

AgentGroup::AgentGroup() {
  static int staticid = 2000;
  id_ = staticid++;

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
}

AgentGroup::AgentGroup(const QList<Agent*>& agentsIn) {
  static int staticid = 0;
  id_ = staticid++;

  // initialize values
  dirty = true;
  dirtyMaxDistance = true;
  members = agentsIn;
  // → delayed center of mass update
  comUpdateTimer.setSingleShot(false);
  comUpdateTimer.setInterval(0);
  connect(&comUpdateTimer, SIGNAL(timeout()), this, SLOT(updateCenterOfMass()));

  // compute center of mass
  updateCenterOfMass();

  // connect signals
  foreach (Agent* agent, members)
    connect(agent, SIGNAL(positionChanged(double, double)), this,
            SLOT(onPositionChanged(double, double)));
}

AgentGroup::AgentGroup(std::initializer_list<Agent*>& agentsIn) {
  static int staticid = 0;
  id_ = staticid++;

  // initialize values
  dirty = true;
  dirtyMaxDistance = true;
  comUpdateTimer.setSingleShot(true);
  comUpdateTimer.setInterval(0);
  connect(&comUpdateTimer, SIGNAL(timeout()), this, SLOT(updateCenterOfMass()));

  // add agents from initializer_list to the member list
  for (Agent* currentAgent : agentsIn) {
    members.append(currentAgent);
    connect(currentAgent, SIGNAL(positionChanged(double, double)), this,
            SLOT(onPositionChanged(double, double)));
  }

  // compute center of mass
  updateCenterOfMass();
}

AgentGroup::~AgentGroup() {}

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
  std::poisson_distribution<int> distribution(CONFIG.group_size_lambda);

  // distribution of group sizes
  QVector<int> sizeDistribution;
  // → create distribution
  int agentCount = agentsIn.count();
  int sizeSum = 0;
  while (sizeSum < agentCount) {
    // randomly draw the group size (Poisson distribution)
    // (don't use group size = 0)
    int groupSize;
    do {
      groupSize = distribution(RNG());
    } while (groupSize == 0);
    // → limit group size to the number of agents left
    groupSize = min(groupSize, agentCount - sizeSum);

    // → record group size
    if (sizeDistribution.size() < groupSize) sizeDistribution.resize(groupSize);
    sizeDistribution[groupSize - 1]++;

    // → update sum over all group sizes
    sizeSum += groupSize;
  }

  // → report group size distribution
  reportSizeDistribution(sizeDistribution);

  if (CONFIG.groups_enabled) {
    // → iterate over all group sizes and create groups accordingly
    //   (start with the largest size to receive contiguous groups)
    for (int groupSize = sizeDistribution.count(); groupSize > 0; --groupSize) {
      // create groups of given size
      for (int groupIter = 0; groupIter < sizeDistribution[groupSize - 1];
           ++groupIter) {
        Agent* groupLeader = unassignedAgents.takeFirst();

        // create a group
        AgentGroup* newGroup = new AgentGroup();
        // and add it to result set
        groups.append(newGroup);

        // add first agent to the group
        Ped::Tvector leaderPosition = groupLeader->getPosition();
        newGroup->addMember(groupLeader);

        // add other agents to group
        QList<QPair<Agent*, double> > distanceList;
        foreach (Agent* potentialMember, unassignedAgents) {
          Ped::Tvector position = potentialMember->getPosition();
          double distance = (leaderPosition - position).length();

          // add potential group member to the list according to the distance
          auto iter = distanceList.begin();
          while (iter < distanceList.end()) {
            if (distance > iter->second)
              break;
            else
              ++iter;
          }
          // → insert candidate
          distanceList.insert(iter, qMakePair(potentialMember, distance));

          // reduce list if necessary
          if (distanceList.size() > groupSize - 1) distanceList.removeFirst();
        }

        // add neighbors to the group
        foreach (const auto& member, distanceList) {
          newGroup->addMember(member.first);

          // don't consider the group member as part of another group
          unassignedAgents.removeOne(member.first);
        }
      }
    }
  }

  return groups;
}

QList<Agent*>& AgentGroup::getMembers() { return members; }

const QList<Agent*>& AgentGroup::getMembers() const { return members; }

bool AgentGroup::addMember(Agent* agentIn) {
  if (members.contains(agentIn)) {
    ROS_DEBUG("AgentGroup: Couldn't add Agent twice!");
    return false;
  }

  // add Agent to the group and mark cache invalid
  members.append(agentIn);
  dirty = true;
  dirtyMaxDistance = true;
  comUpdateTimer.start();

  // connect signals
  connect(agentIn, SIGNAL(positionChanged(double, double)), this,
          SLOT(onPositionChanged(double, double)));

  // inform users
  emit memberAdded(agentIn->getId());

  return true;
}

bool AgentGroup::removeMember(Agent* agentIn) {
  bool hasRemovedMember = members.removeOne(agentIn);

  // mark cache invalid, if the agent has been removed
  if (hasRemovedMember == true) {
    // disconnect signals
    disconnect(agentIn, SIGNAL(positionChanged(double, double)), this,
               SLOT(onPositionChanged(double, double)));

    // invalidate cache and schedule update
    dirty = true;
    dirtyMaxDistance = true;
    comUpdateTimer.start();

    // inform users
    emit memberRemoved(agentIn->getId());

    return true;
  } else {
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
  foreach (Agent* agent, members)
    connect(agent, SIGNAL(positionChanged(double, double)), this,
            SLOT(onPositionChanged(double, double)));

  // inform users
  // TODO - we need to get away from using signals
  emit membersChanged();

  return true;
}

bool AgentGroup::isEmpty() const { return members.isEmpty(); }

int AgentGroup::memberCount() const { return members.count(); }

Ped::Tvector AgentGroup::getCenterOfMass() const {
  // check cache
  if (dirty) {
    // update cache
    AgentGroup* nonConstThis = const_cast<AgentGroup*>(this);
    nonConstThis->updateCenterOfMass();
  }

  return cacheCoM;
}

Ped::Tvector AgentGroup::updateCenterOfMass() {
  if (!dirty) return cacheCoM;

  // compute center of mass
  Ped::Tvector com;
  foreach (const Agent* member, members) { com += member->getPosition(); }

  int groupSize = members.size();
  com /= groupSize;

  // set cache value
  cacheCoM = com;

  // mark the cache as valid
  dirty = false;

  return cacheCoM;
}

void AgentGroup::setRecollect(bool recollectIn) {
  if (recollectIn) {
    // check whether recollecting mode has already been activated
    if (recollecting) return;

    ROS_DEBUG("AgentGroup needs to recollect! (%s)",
              toString().toStdString().c_str());
    recollecting = true;
  } else {
    // check whether recollecting mode hasn't been activated
    if (!recollecting) return;

    ROS_DEBUG("AgentGroup finished recollecting! (%s)",
              toString().toStdString().c_str());
    recollecting = false;
  }
}

bool AgentGroup::isRecollecting() const { return recollecting; }

double AgentGroup::getMaxDistance() {
  if (dirty || dirtyMaxDistance) updateMaxDistance();

  return cacheMaxDistance;
}

void AgentGroup::updateMaxDistance() {
  Ped::Tvector com = getCenterOfMass();
  double maxDistance = 0;
  foreach (Agent* agent, members) {
    double distance = (com - agent->getPosition()).length();
    if (distance > maxDistance) maxDistance = distance;
  }
  cacheMaxDistance = maxDistance;
  dirtyMaxDistance = false;
}

void AgentGroup::reportSizeDistribution(
    const QVector<int>& sizeDistributionIn) {
  QString sizeDistributionString;
  int groupSize = 1;
  foreach (int count, sizeDistributionIn) {
    sizeDistributionString += tr(" %1: %2;").arg(groupSize).arg(count);
    groupSize++;
  }
  ROS_DEBUG("Group Size Distribution: %s",
            sizeDistributionString.toStdString().c_str());
}

QString AgentGroup::toString() const {
  QString agentString;
  bool firstMember = true;
  foreach (Agent* agent, members) {
    if (!firstMember) agentString += ", ";
    agentString += agent->toString();
    firstMember = false;
  }

  return tr("AgentGroup (CoM: @%1,%2; Members:%3)")
      .arg(cacheCoM.x)
      .arg(cacheCoM.y)
      .arg(agentString);
}

//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2004 by Christian Gloor
//

#include "ped_tree.h"
#include "ped_agent.h"
#include "ped_scene.h"

#include <cassert>
#include <cstddef>

using namespace std;

/// Description: set intial values
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::Ttree(Ped::Tscene* pscene, int pdepth, double px, double py,
                  double pw, double ph)
    : scene(pscene) {
  // more initializations here. not really necessary to put them into the
  // initializator list, too.
  isleaf = true;
  x = px;
  y = py;
  w = pw;
  h = ph;
  depth = pdepth;
  tree1 = NULL;
  tree2 = NULL;
  tree3 = NULL;
  tree4 = NULL;
}

/// Destructor. Deleted this node and all its children. If there are any agents
/// left, they are removed first (not deleted).
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::~Ttree() { clear(); }

void Ped::Ttree::clear() {
  if (isleaf) {
    agents.clear();
  } else {
    tree1->clear();
    delete tree1;
    tree2->clear();
    delete tree2;
    tree3->clear();
    delete tree3;
    tree4->clear();
    delete tree4;
    isleaf = true;
  }
}

/// Adds an agent to the tree. Searches the right node and adds the agent there.
/// If there are too many agents at that node allready, a new child is created.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a The agent to add
void Ped::Ttree::addAgent(const Ped::Tagent* a) {
  if (isleaf) {
    agents.insert(a);
    scene->treehash[a] = this;
  } else {
    if ((a->getx() >= x + w / 2) && (a->gety() >= y + h / 2))
      tree3->addAgent(a);  // 3
    if ((a->getx() <= x + w / 2) && (a->gety() <= y + h / 2))
      tree1->addAgent(a);  // 1
    if ((a->getx() >= x + w / 2) && (a->gety() <= y + h / 2))
      tree2->addAgent(a);  // 2
    if ((a->getx() <= x + w / 2) && (a->gety() >= y + h / 2))
      tree4->addAgent(a);  // 4
  }

  if (agents.size() > 8) {
    isleaf = false;
    addChildren();
    while (!agents.empty()) {
      const Ped::Tagent* a = (*agents.begin());
      if ((a->getx() >= x + w / 2) && (a->gety() >= y + h / 2))
        tree3->addAgent(a);  // 3
      if ((a->getx() <= x + w / 2) && (a->gety() <= y + h / 2))
        tree1->addAgent(a);  // 1
      if ((a->getx() >= x + w / 2) && (a->gety() <= y + h / 2))
        tree2->addAgent(a);  // 2
      if ((a->getx() <= x + w / 2) && (a->gety() >= y + h / 2))
        tree4->addAgent(a);  // 4
      agents.erase(a);
    }
  }
}

/// A little helper that adds child nodes to this node
/// \author  chgloor
/// \date    2012-01-28
void Ped::Ttree::addChildren() {
  tree1 = new Ped::Ttree(scene, depth + 1, x, y, w / 2, h / 2);
  tree2 = new Ped::Ttree(scene, depth + 1, x + w / 2, y, w / 2, h / 2);
  tree3 = new Ped::Ttree(scene, depth + 1, x + w / 2, y + h / 2, w / 2, h / 2);
  tree4 = new Ped::Ttree(scene, depth + 1, x, y + h / 2, w / 2, h / 2);
}

Ped::Ttree* Ped::Ttree::getChildByPosition(double xIn, double yIn) {
  if ((xIn <= x + w / 2) && (yIn <= y + h / 2)) return tree1;
  if ((xIn >= x + w / 2) && (yIn <= y + h / 2)) return tree2;
  if ((xIn >= x + w / 2) && (yIn >= y + h / 2)) return tree3;
  if ((xIn <= x + w / 2) && (yIn >= y + h / 2)) return tree4;

  // this should never happen
  return NULL;
}

/// Updates the tree structure if an agent moves. Removes the agent and places
/// it again, if outside boundary.
/// If an this happens, this is O(log n), but O(1) otherwise.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a the agent to update
void Ped::Ttree::moveAgent(const Ped::Tagent* a) {
  if ((a->getx() < x) || (a->getx() > (x + w)) || (a->gety() < y) ||
      (a->gety() > (y + h))) {
    scene->placeAgent(a);
    agents.erase(a);
  }
}

bool Ped::Ttree::removeAgent(const Ped::Tagent* a) {
  if (isleaf) {
    size_t removedCount = agents.erase(a);
    return (removedCount > 0);
  } else {
    return getChildByPosition(a->getx(), a->gety())->removeAgent(a);
  }
}

/// Checks if this tree node has not enough agents in it to justify more child
/// nodes. It does this by checking all
/// child nodes, too, recursively. If there are not enough children, it moves
/// all the agents into this node, and deletes the child nodes.
/// \author  chgloor
/// \date    2012-01-28
/// \return  the number of agents in this and all child nodes.
int Ped::Ttree::cut() {
  if (isleaf) return agents.size();

  int count = 0;
  count += tree1->cut();
  count += tree2->cut();
  count += tree3->cut();
  count += tree4->cut();
  if (count < 5) {
    assert(tree1->isleaf == true);
    assert(tree2->isleaf == true);
    assert(tree3->isleaf == true);
    assert(tree4->isleaf == true);
    agents.insert(tree1->agents.begin(), tree1->agents.end());
    agents.insert(tree2->agents.begin(), tree2->agents.end());
    agents.insert(tree3->agents.begin(), tree3->agents.end());
    agents.insert(tree4->agents.begin(), tree4->agents.end());
    isleaf = true;
    for (const Tagent* agent : agents) scene->treehash[agent] = this;

    delete tree1;
    delete tree2;
    delete tree3;
    delete tree4;
  }
  return count;
}

/// Returns the set of agents that is stored within this tree node
/// \author  chgloor
/// \date    2012-01-28
/// \return  The set of agents
/// \todo This might be not very efficient, since all childs are checked, too.
/// And then the set (set of pointer, but still) is being copied around.
set<const Ped::Tagent*> Ped::Ttree::getAgents() const {
  if (isleaf) return agents;

  // fill a temporary output list with the agents
  vector<const Ped::Tagent*> tempList;
  getAgents(tempList);

  // convert list to set
  return set<const Ped::Tagent*>(tempList.begin(), tempList.end());
}

void Ped::Ttree::getAgents(vector<const Ped::Tagent*>& outputList) const {
  if (isleaf) {
    for (const Ped::Tagent* currentAgent : agents)
      outputList.push_back(currentAgent);
  } else {
    tree1->getAgents(outputList);
    tree2->getAgents(outputList);
    tree3->getAgents(outputList);
    tree4->getAgents(outputList);
  }
}

/// Checks if a point x/y is within the space handled by the tree node, or
/// within a given radius r
/// \author  chgloor
/// \date    2012-01-29
/// \return  true if the point is within the space
/// \param   px The x co-ordinate of the point
/// \param   py The y co-ordinate of the point
/// \param   pr The radius
bool Ped::Ttree::intersects(double px, double py, double pr) const {
  if (((px + pr) > x) && ((px - pr) < (x + w)) && ((py + pr) > y) &&
      ((py - pr) < (y + h)))
    return true;  // x+-r/y+-r is inside
  else
    return false;
}

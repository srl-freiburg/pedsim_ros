//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#include "ped_scene.h"
#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_tree.h"
#include "ped_waypoint.h"

#include <algorithm>
#include <cstddef>
#include <stack>

using namespace std;

/// Default constructor. If this constructor is used, there will be no quadtree
/// created.
/// This is faster for small scenarios or less than 1000 Tagents.
Ped::Tscene::Tscene() : tree(NULL) {}

/// Constructor used to create a quadtree statial representation of the Tagents.
/// Use this
/// constructor when you have a sparsely populated world with many agents
/// (>1000).
/// The agents must not be outside the boundaries given here. If in doubt, use
/// an initial
//// boundary that is way to big.
/// \todo    Get rid of that limitation. A dynamical outer boundary algorithm
/// would be nice.
/// \param left is the left side of the boundary
/// \param top is the upper side of the boundary
/// \param width is the total width of the boundary. Basically from left to
/// right.
/// \param height is the total height of the boundary. Basically from top to
/// down.
Ped::Tscene::Tscene(double left, double top, double width, double height) {
  tree = new Ped::Ttree(this, 0, left, top, width, height);
}

/// Destructor
Ped::Tscene::~Tscene() { delete tree; }

void Ped::Tscene::clear() {
  // clear tree
  treehash.clear();
  tree->clear();

  // remove all agents
  for (Ped::Tagent* currentAgent : agents) delete currentAgent;
  agents.clear();

  // remove all obstacles
  for (Ped::Tobstacle* currentObstacle : obstacles) delete currentObstacle;
  obstacles.clear();

  // remove all waypoints
  for (Ped::Twaypoint* currentWaypoint : waypoints) delete currentWaypoint;
  waypoints.clear();
}

/// Used to add a Tagent to the Tscene.
/// \warning addAgent() does call Tagent::assignScene() to assign itself to the
/// agent.
/// \param   *a A pointer to the Tagent to add.
void Ped::Tscene::addAgent(Ped::Tagent* a) {
  // add agent to scene
  // (take responsibility for object deletion)
  agents.push_back(a);
  a->assignScene(this);
  if (tree != NULL) tree->addAgent(a);
}

/// Used to add a Tobstacle to the Tscene.
/// \param   *o A pointer to the Tobstacle to add.
/// \note    Obstacles added to the Scene are not deleted if the Scene is
/// destroyed. The reason for this is because they could be member of another
/// Scene theoretically.
void Ped::Tscene::addObstacle(Ped::Tobstacle* o) {
  // add obstacle to scene
  // (take responsibility for object deletion)
  obstacles.push_back(o);
}

void Ped::Tscene::addWaypoint(Ped::Twaypoint* w) {
  // add waypoint to scene
  // (take responsibility for object deletion)
  waypoints.push_back(w);
}

bool Ped::Tscene::removeAgent(Ped::Tagent* a) {
  // find position of agent in agent vector
  vector<Tagent*>::iterator agentIter = find(agents.begin(), agents.end(), a);

  // check whether the agent was found
  if (agentIter == agents.end()) return false;

  // remove agent as potential neighbor
  for (Tagent* currentAgent : agents) currentAgent->removeAgentFromNeighbors(a);

  // remove agent from the tree
  if (tree != NULL) tree->removeAgent(a);

  // remove agent from the scene and delete it, report succesful removal
  agents.erase(agentIter);
  delete a;
  return true;
}

bool Ped::Tscene::removeObstacle(Ped::Tobstacle* o) {
  // find position of obstacle in obstacle vector
  vector<Tobstacle*>::iterator obstacleIter =
      find(obstacles.begin(), obstacles.end(), o);

  // check whether the obstacle was found
  if (obstacleIter == obstacles.end()) return false;

  // remove obstacle from the scene and delete it, report succesful removal
  obstacles.erase(obstacleIter);
  delete o;
  return true;
}

bool Ped::Tscene::removeWaypoint(Ped::Twaypoint* w) {
  // find position of waypoint in waypoint vector
  vector<Twaypoint*>::iterator waypointIter =
      find(waypoints.begin(), waypoints.end(), w);

  // check whether the waypoint was found
  if (waypointIter == waypoints.end()) return false;

  // remove waypoint from the scene and delete it, report succesful removal
  waypoints.erase(waypointIter);
  delete w;

  return true;
}

/// This is a convenience method. It calls Ped::Tagent::move(double h) for all
/// agents in the Tscene.
/// \param   h This tells the simulation how far the agents should proceed.
/// \see     Ped::Tagent::move(double h)
void Ped::Tscene::moveAgents(double h) {
  // first update states
  for (Tagent* agent : agents) agent->updateState();

  // then update forces
  for (Tagent* agent : agents) agent->computeForces();

  // finally move agents according to their forces
  for (Tagent* agent : agents) agent->move(h);
}

/// Internally used to update the quadtree.
void Ped::Tscene::placeAgent(const Ped::Tagent* agentIn) {
  if (tree != NULL) tree->addAgent(agentIn);
}

/// Moves a Tagent within the tree structure. The new position is taken from the
/// agent. So it
/// basically updates the tree structure for that given agent.
/// Ped::Tagent::move(double h) calls
/// this method automatically.
/// \param   *agentIn the agent to move.
void Ped::Tscene::moveAgent(const Ped::Tagent* agentIn) {
  if (tree != NULL) treehash[agentIn]->moveAgent(agentIn);
}

/// This triggers a cleanup of the tree structure. Unused leaf nodes are
/// collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20
/// timestep.
/// \date    2012-01-28
void Ped::Tscene::cleanup() {
  if (tree != NULL) tree->cut();
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents
/// (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Tscene::getNeighbors(double x, double y,
                                                  double dist) const {
  // if there is no tree, return all agents
  if (tree == NULL)
    return set<const Ped::Tagent*>(agents.begin(), agents.end());

  // create the output list
  vector<const Ped::Tagent*> neighborList;
  getNeighbors(neighborList, x, y, dist);

  // copy the neighbors to a set
  return set<const Ped::Tagent*>(neighborList.begin(), neighborList.end());
}

void Ped::Tscene::getNeighbors(vector<const Ped::Tagent*>& neighborList,
                               double x, double y, double dist) const {
  stack<Ped::Ttree*> treestack;

  treestack.push(tree);
  while (!treestack.empty()) {
    Ped::Ttree* t = treestack.top();
    treestack.pop();
    if (t->isleaf) {
      t->getAgents(neighborList);
    } else {
      if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
      if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
      if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
      if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
    }
  }
}

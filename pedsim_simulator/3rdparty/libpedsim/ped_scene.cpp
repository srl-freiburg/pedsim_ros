//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#include "ped_scene.h"
#include "ped_agent.h"
#include "ped_tree.h"

#include <stack>

using namespace std;


/// Default constructor. If this constructor is used, there will be no quadtree created. 
/// This is faster for small scenarios or less than 1000 Tagents.
/// \date    2012-01-17
Ped::Tscene::Tscene() : tree(NULL) {};


/// Constructor used to create a quadtree statial representation of the Tagents. Use this
/// constructor when you have a sparsely populated world with many agents (>1000). 
/// The agents must not be outside the boundaries given here. If in doubt, use an initial
//// boundary that is way to big. 
/// \todo    Get rid of that limitation. A dynamical outer boundary algorithm would be nice. 
/// \date    2012-01-17
/// \param left is the left side of the boundary
/// \param up is the upper side of the boundary
/// \param width is the total width of the boundary. Basically from left to right.
/// \param height is the total height of the boundary. Basically from up to down.
Ped::Tscene::Tscene(double left, double up, double width, double height) {
    tree = new Ped::Ttree(this, 0, left, up, width, height);
}


/// Destructor
/// \date    2012-02-04
Ped::Tscene::~Tscene() {
    if (tree != NULL) delete(tree); // I know that delete(NULL) always succeeds. Still like this better.
}


/// Used to add a Tagent to the Tscene. 
/// \date    2012-01-17
/// \warning addAgent() does call Tagent::assignScene() to assign itself to the agent.
/// \param   *a A pointer to the Tagent to add. 
/// \note    The Tagents* given to addAgent() are not const (i.e. not const Tagent*) because of moveAgents(double h). It obviously modifies the agents. Agents added to the Scene are not deleted if the Scene is destroyed. The reason for this is because they could be member of another Scene theoretically. 
void Ped::Tscene::addAgent(Ped::Tagent *a) {
    agent.push_back(a);
    a->assignScene(this);
    if (tree != NULL) tree->addAgent(a);
}


/// This is a convenience method. It calls Ped::Tagent::move(double h) for all agents in the Tscene.
/// \date    2012-02-03
/// \param   h This tells the simulation how far the agents should proceed. 
/// \see     Ped::Tagent::move(double h)
void Ped::Tscene::moveAgents(double h) {
    for (vector<Tagent*>::iterator iter = agent.begin(); iter != agent.end(); ++iter) {
        Tagent *a = (*iter);
        a->move(h);
    }
}


/// Used to add a Tobstacle to the Tscene.
/// \date    2012-01-17
/// \param   *o A pointer to the Tobstacle to add.
/// \note    Obstacles added to the Scene are not deleted if the Scene is destroyed. The reason for this is because they could be member of another Scene theoretically. 
void Ped::Tscene::addObstacle(Ped::Tobstacle *o) {
    obstacle.push_back(o);
}


/// Internally used to update the quadtree. 
/// \date    2012-01-28
/// \param   
void Ped::Tscene::placeAgent(const Ped::Tagent *a) {
    if (tree != NULL) tree->addAgent(a);
}


/// Moves a Tagent within the tree structure. The new position is taken from the agent. So it 
/// basically updates the tree structure for that given agent. Ped::Tagent::move(double h) calls 
/// this method automatically. 
/// \date    2012-01-28
/// \param   *a the agent to move. 
void Ped::Tscene::moveAgent(const Ped::Tagent *a) {
    if (tree != NULL) treehash[a]->moveAgent(a);
}

/// Kill an agent
void Ped::Tscene::killAgent(const Ped::Tagent *a) {
    // if (tree != NULL) {
    //     tree->agents.erase(a);

    //     // also remove it from the vector of agents
    //     agent.erase(std::remove(agent.begin(), agent.end(), a), agent.end());
    // }
}

/// This triggers a cleanup of the tree structure. Unused leaf nodes are collected in order to
/// save memory. Ideally cleanup() is called every second, or about every 20 timestep.
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
/// \param   dist the distance around x/y that will be saerched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Tscene::getNeighbors(double x, double y, double dist) const {
    stack<Ped::Ttree*> treestack;
    set<const Ped::Tagent*> neighbors;

    if (tree == NULL) {
        neighbors.insert(agent.begin(), agent.end());
    } else {
        treestack.push(tree);
        while(!treestack.empty()) {
            Ped::Ttree *t = treestack.top();
            treestack.pop();
            if (t->isleaf) {
                set<const Ped::Tagent*> a = t->getAgents();
                neighbors.insert(a.begin(), a.end());
            } else {
                if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
                if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
                if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
                if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
            }
        }
    }
    return neighbors;
}

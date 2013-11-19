//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2004 by Christian Gloor
//     

#include "ped_agent.h"                         
#include "ped_scene.h"
#include "ped_tree.h"

#include <iostream>
//#include <algorithm>

#include <cassert>

using namespace std;


/// Description: set intial values
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::Ttree(Ped::Tscene *pscene, int pdepth, double px, double py, double pw, double ph) : scene(pscene) {
	// more initializations here. not really necessary to putm the into the initializator list, too. 
	agentcount = 0;
	isleaf = true;
	x = px; y = py; w = pw; h = ph;
	depth = pdepth;
};


/// Destructor. Deleted this node and all its children. If there are any agents left, they are removed first (not deleted).
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::~Ttree() {
	agents.clear();
	if (isleaf == false) {
		delete(tree1);
		delete(tree2);
		delete(tree3);
		delete(tree4);
		isleaf = true; // not really necessary here, but added for consistency
	}
}


/// Adds an agent to the tree. Searches the right node and adds the agent there. 
/// If there are too many agents at that node allready, a new child is created. 
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a The agent to add
void Ped::Ttree::addAgent(const Ped::Tagent *a) {
	if (isleaf) {
		agents.insert(a);
		scene->treehash[a] = this;
	} else {
		if ((a->getx() >= x+0.5f*w) && (a->gety() >= y+0.5f*h)) tree3->addAgent(a); // 3
		if ((a->getx() <= x+0.5f*w) && (a->gety() <= y+0.5f*h)) tree1->addAgent(a); // 1
		if ((a->getx() >= x+0.5f*w) && (a->gety() <= y+0.5f*h)) tree2->addAgent(a); // 2
		if ((a->getx() <= x+0.5f*w) && (a->gety() >= y+0.5f*h)) tree4->addAgent(a); // 4
	}
	if (agents.size() > 8) {
		isleaf = false;
		addChildren();
		while (!agents.empty()) {
			const Ped::Tagent *a = (*agents.begin());
			if ((a->getx() >= x+0.5f*w) && (a->gety() >= y+0.5f*h)) tree3->addAgent(a); // 3
			if ((a->getx() <= x+0.5f*w) && (a->gety() <= y+0.5f*h)) tree1->addAgent(a); // 1
			if ((a->getx() >= x+0.5f*w) && (a->gety() <= y+0.5f*h)) tree2->addAgent(a); // 2
			if ((a->getx() <= x+0.5f*w) && (a->gety() >= y+0.5f*h)) tree4->addAgent(a); // 4
			agents.erase(a);
		}
	}
}


/// A little helper that adds child nodes to this node
/// \author  chgloor
/// \date    2012-01-28
void Ped::Ttree::addChildren() {
	tree1 = new Ped::Ttree(scene, depth+1, x, y, 0.5f*w, 0.5f*h);
	tree2 = new Ped::Ttree(scene, depth+1, x+0.5f*w, y, 0.5f*w, 0.5f*h);
	tree3 = new Ped::Ttree(scene, depth+1, x+0.5f*w, y+0.5f*h, 0.5f*w, 0.5f*h);
	tree4 = new Ped::Ttree(scene, depth+1, x, y+0.5f*h, 0.5f*w, 0.5f*h);
}


/// Updates the tree structure if an agent moves. Removes the agent and places it again, if outside boundary. 
/// If an this happens, this is O(log n), but O(1) otherwise.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a the agent to update 
void Ped::Ttree::moveAgent(const Ped::Tagent *a) {
	if ((a->getx() < x) || (a->getx() > (x+w)) || (a->gety() < y) || (a->gety() > (y+h))) {
		scene->placeAgent(a);		
		agents.erase(a);
	}	
}


/// Checks if this tree node has not enough agents in it to justify more child nodes. It does this by checking all
/// child nodes, too, recursively. If there are not enough children, it moves all the agents into this node, and deletes the child nodes. 
/// \author  chgloor
/// \date    2012-01-28
/// \return  the number of agents in this and all child nodes. 
int Ped::Ttree::cut() {
	if (isleaf) {
		return agents.size();		
	} else {
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
			for (set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
				const Tagent *a = (*it);
				scene->treehash[a] = this;
			}
			delete(tree1);
			delete(tree2);
			delete(tree3);
			delete(tree4);
		}
		return count;
	}
}


/// Returns the set of agents that is stored within this tree node
/// \author  chgloor
/// \date    2012-01-28
/// \return  The set of agents
/// \todo This might be not very efficient, since all childs are checked, too. And then the set (set of pointer, but still) is being copied around. 
set<const Ped::Tagent*> Ped::Ttree::getAgents() const {
	set<const Ped::Tagent*> ta;
	if (isleaf) {
		return agents;
	} else {
		set<const Ped::Tagent*> t1 = tree1->getAgents();		
		set<const Ped::Tagent*> t2 = tree2->getAgents();		
		set<const Ped::Tagent*> t3 = tree3->getAgents();		
		set<const Ped::Tagent*> t4 = tree4->getAgents();		
		ta.insert(t1.begin(), t1.end());
		ta.insert(t2.begin(), t2.end());
		ta.insert(t3.begin(), t3.end());
		ta.insert(t4.begin(), t4.end());
	}
	return ta;
}


/// Checks if a point x/y is within the space handled by the tree node, or within a given radius r
/// \author  chgloor
/// \date    2012-01-29
/// \return  true if the point is within the space
/// \param   px The x co-ordinate of the point
/// \param   py The y co-ordinate of the point
/// \param   pr The radius 
bool Ped::Ttree::intersects(double px, double py, double pr) const {
	if (((px+pr) > x) && ((px-pr) < (x+w)) && ((py+pr) > y) && ((py-pr) < (y+h))) return true; // x+-r/y+-r is inside
	return false;
}

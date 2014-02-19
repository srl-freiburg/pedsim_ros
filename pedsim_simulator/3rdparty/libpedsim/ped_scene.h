
//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#ifndef _ped_scene_h_
#define _ped_scene_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <set>
#include <vector>
#include <map>
#include <algorithm>

namespace Ped {

class Tagent;
class Tobstacle;
class Ttree;

using namespace std;

/// The Tscene class contains the spatial representation of the "world" the agents live in.
/// Theoretically, in a continuous model, there are no boundaries to the size of the world.
/// Agents know their position (the x/y co-ordinates). However, to find the nearest neighbors of
/// an agent, it makes sense to put them in some kind of "boxes". In this implementation, the
/// infinite world is divided by a dynamic quadtree structure. There are some  CPU cycles
/// required to update the structure with each agent position change. But the gain in looking
/// up the neighbors is worth this. The quadtree structure only needs to be changed when an
/// agent leaves its box, which migh only happen every 100th or 1000th timestep, depending on
/// the box size.
/// The Tscene class needs an outer boundary in order to construct the initial box of the
/// quadtree. Agents are not allowd to go outside that boundary. If you do not know how far
/// they will walk, choose a rather big boundary box. The quadtree algorythm will dynamically
/// assign smaller sub-boxes within if required.
/// If all (most) agents walk out of a box, it is no longer needed. It can be colleted. If
/// there are some agents left, they will be assigned to the box above in the hierarchy. You must
/// trigger this collection process periodically by calling cleanup() manually
/// \author  chgloor
/// \date    2010-02-12
class LIBEXPORT Tscene {
    friend class Ped::Tagent;
    friend class Ped::Ttree;

public:
    Tscene();
    Tscene(double left, double up, double width, double height);
    virtual ~Tscene();

    virtual void addAgent(Tagent *a);
    virtual void addObstacle(Tobstacle *o);

    virtual void cleanup();
    virtual void moveAgents(double h);

    set<const Ped::Tagent*> getNeighbors(double x, double y, double dist) const;
    const vector<Tagent*>& getAllAgents() { return agent; };

    void moveAgent(const Ped::Tagent *a);
    void killAgent(const Ped::Tagent *a);

private:
    vector<Tagent*> agent;
    vector<Tobstacle*> obstacle;
    map<const Ped::Tagent*, Ttree*> treehash;

    void placeAgent(const Ped::Tagent *a);


    // Copy constructor (PRIVATE)
    Tscene(const Tscene& source) {
        // if used: copy the *tree here
    };

    // Assignment operator (PRIVATE)
    Tscene& operator= (const Tscene& source) {
        if (this == &source) return *this;
        // if used: copy the *tree here
        return *this;
    }


protected:
    Ttree *tree;

};
};
#endif

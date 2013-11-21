//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 20012 by Christian Gloor
//                              


#include "math.h"

#include "ped_vector.h"
#include "ped_agent.h"
#include "ped_obstacle.h"

#include <vector>
#include <stdlib.h> // rand

using namespace std;


/// Default Constructor 
/// \date    2003-12-29
Ped::Tagent::Tagent() {
    static int staticid = 0;
    id = staticid++;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    v.x = 0;
    v.y = 0;
    v.z = 0;
    hasreacheddestination = true;
    destination = NULL;
    lastdestination = NULL;
    birth_waypoint = NULL;
    death_waypoint = NULL;
    follow = -1;
    // vmax = 1.5 + 1.5*(double)rand()/(double)RAND_MAX; // in m/s between 2.0 and 4.0
    // vmax = 0.5 + 2.0*(double)rand()/(double)RAND_MAX; // in m/s between 2.0 and 4.0
    vmax = 1.0; // For verification tests

    mlLookAhead = false;

    factorsocialforce = 2.1f;
    factorobstacleforce = 10.0f;
    factordesiredforce = 1.0f;
    factorlookaheadforce = 1.0f;

    timestep = 0;
}


/// Default destructor
/// \date    2012-02-04
Ped::Tagent::~Tagent() {
}


/// Adds a TWaypoint to an agent's list of waypoints. Twaypoints are stored in a 
/// cyclic queue, the one just visited is pushed to the back again. There will be a 
/// flag to change this behavior soon.
/// \todo Add a flag to change the waypoint queue behavior of the Tagents.
/// \author  chgloor
/// \date    2012-01-19
void Ped::Tagent::addWaypoint(Twaypoint *wp) {
    destinations.push(wp);

    if (wp->gettype() == Ped::Twaypoint::TYPE_BIRTH) {
        birth_waypoint = wp;
    }

    if (wp->gettype() == Ped::Twaypoint::TYPE_DEATH) {
        death_waypoint = wp;
    }
}


/// Assigns a Tscene to the agent. Tagent uses this to iterate over all obstacles and other agents in a scene.
/// The scene will invoke this function when Tscene::addAgent() is called.  
/// \date    2012-01-17
/// \warning Bad things will happen if the agent is not assigned to a scene. But usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene(Ped::Tscene *s) {
    scene = s;
}


/// Sets the agent ID this agent has to follow. If set, the agent will ignore 
/// its assigned waypoints and just follow the other agent.
/// \date    2012-01-08
/// \param   id is the agent to follow (must exist, obviously)
/// \todo    Add a method that takes a Tagent* as argument
void Ped::Tagent::setFollow(int id) {
    follow = id;
}


/// Gets the ID of the agent this agent is following.
/// \date    2012-01-18
/// \return  int, the agent id of the agent
/// \todo    Add a method that returns a Tagent* 
int Ped::Tagent::getFollow() const {
    return follow;
}


/// Sets the maximum velocity of an agent (vmax). Even if pushed by other agents, it will not move faster than this.
/// \date    2012-01-08
/// \param   pvmax The maximum velocity. In scene units per timestep, multiplied by the simulation's precision h.
void Ped::Tagent::setVmax(double pvmax) {
    vmax = pvmax;
}


/// Sets the agent's position. This, and other getters returning coordinates, will eventually changed to returning a Tvector.
/// \date    2004-02-10
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition(double px, double py, double pz) {
    p.x = px; p.y = py; p.z = pz;
};


/// Sets the factor by which the social force is multiplied. Values between 0 and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorsocialforce(double f) {factorsocialforce = f; };

/// Sets the factor by which the obstacle force is multiplied. Values between 0 and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorobstacleforce(double f) {factorobstacleforce = f; };

/// Sets the factor by which the desired force is multiplied. Values between 0 and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactordesiredforce(double f) {factordesiredforce = f; };

/// Sets the factor by which the look ahead force is multiplied. Values between 0 and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorlookaheadforce(double f) {factorlookaheadforce = f; };


/// Calculates the social force between this agent and all the other agents belonging to the same scene.
/// It iterates over all agents inside the scene, has therefore the complexity O(N^2). A better
/// agent storing structure in Tscene would fix this. But for small (less than 10000 agents) scenarios, this is just fine. 
/// \date    2012-01-17
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce() const {
    Ped::Tvector s;
    //	for (AgentIterator iter = scene->agent.begin(); iter!=scene->agent.end(); ++iter) {
    for (set<const Ped::Tagent*>::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter) {
        Ped::Tvector f;
        if (((*iter)->id != id)) {
            if ((fabs(p.x-(*iter)->p.x) < 10.0) && (fabs(p.y-(*iter)->p.y) < 10.0)) { // quick dist check
                Ped::Tvector d;
                d.x = (*iter)->p.x - p.x;
                d.y = (*iter)->p.y - p.y;
                d.z = (*iter)->p.z - p.z;
                double dist2 = (d.x * d.x + d.y * d.y + d.z * d.z);
                double expdist = exp(sqrt(dist2)-1);
                if ((dist2 > 0.000004) && (dist2 < 400)) { // 2cm- 20m distance
                    f.x = -d.x/expdist;
                    f.y = -d.y/expdist;
                }
                s.x += f.x;
                s.y += f.y;
                s.z += f.z;
            }
        }
    }
    return s;
}


/// Calculates the force between this agent and the nearest obstacle in this scene.
/// Iterates over all obstacles == O(N). 
/// \date    2012-01-17
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::obstacleForce() const {
    Ped::Tvector o;
    double mindisto2 = 99999; // obstacle with is closest only  --chgloor 2012-01-12
    double mindox = 0;
    double mindoy = 0;

    for (vector<Tobstacle*>::iterator iter = scene->obstacle.begin(); iter!=scene->obstacle.end(); ++iter) {
        Ped::Tvector ov = (*iter)->obstacleforce(p.x, p.y);
        double dox = p.x - ov.x;
        double doy = p.y - ov.y;
        double disto2 = (dox * dox + doy * doy);  // dist2 = distanz im quadrat
        if ((disto2 < mindisto2) && (disto2 > 0.000004)) { // 2cm - inf distance
            mindisto2 = disto2;
            mindox = dox;
            mindoy = doy;
        }
    }
    double oaxyf = exp(sqrt(mindisto2)-1);
    o.x = mindox/oaxyf;
    o.y = mindoy/oaxyf;
    return o;
}


/// Calculates the force between this agent and the next assigned waypoint. 
/// If the waypoint has been reached, the next waypoint in the list will be selected.
/// At the moment, a visited waypoint is pushed back to the end of the list, which
/// means that the agents will visit all the waypoints over and over again. In a later
/// release, this behavior can be controlled by a flag.
/// \date    2012-01-17
/// \todo    move this destination handling into a separate method called by move(). then mark this method as const
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce() {
    Ped::Tvector e;

    if (follow >= 0) {
        bool reached;
        Twaypoint d(scene->agent.at(follow)->getx(), scene->agent.at(follow)->gety(), 0);
        d.setType(Ped::Twaypoint::TYPE_POINT);
        Ped::Tvector ef = d.getForce(p.x, p.y, 0, 0, &reached);
        e.x = ef.x * vmax; // walk with full speed if nothing else affects me
        e.y = ef.y * vmax;
        return e;
    }

    // waypoint management (fetch new destination if available)
    // consider: replace hasreacheddestination with destination==NULL
    if ((hasreacheddestination == true) && (destinations.size() > 0)) {
        lastdestination = destination;
        destination = destinations.front();
        destinations.pop();
        hasreacheddestination = false;
    }


    if (destination != NULL) {
        bool reached;
        Ped::Tvector ef;
        if (lastdestination == NULL) { // create a temporary destination of type point, since no normal from last dest is available
            Twaypoint d(destination->getx(), destination->gety(), destination->getr());
            d.setType(Ped::Twaypoint::TYPE_POINT);
            ef = d.getForce(p.x, p.y, 0, 0, &reached);
        } else {
            ef = destination->getForce(p.x, p.y, lastdestination->getx(), lastdestination->gety(), &reached);
        }

        e.x = ef.x * vmax; // walk with full speed if nothing else affects me
        e.y = ef.y * vmax;

        if (hasreacheddestination == false) {
            if (reached == true) {
                hasreacheddestination = true;
                destinations.push(destination); // round queue
                destination == NULL;
            }
        }
    }

    return e;
}


/// Calculates the mental layer force of the strategy "look ahead". It is implemented here in the physical layer
/// because of performance reasons. It iterates over all Tagents in the Tscene, complexity O(N^2).
/// \date    2012-01-17
/// \return  Tvector: the calculated force
/// \param   e is a vector defining the direction in which the agent should look ahead to. Usually, this is the direction he wants to walk to.
Ped::Tvector Ped::Tagent::lookaheadForce(Ped::Tvector e) const {
    Ped::Tvector lf;

    int lookforwardcount = 0;
    //	for (AgentIterator iter = scene->agent.begin(); iter!=scene->agent.end(); iter++) {  // iterate over all agents == O(N^2) :(
    for (set<const Ped::Tagent*>::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter) {
        if (((*iter)->id != id)) {
            double distancex = (*iter)->p.x - p.x;
            double distancey = (*iter)->p.y - p.y;
            // double dist2 = (distancex * distancex + distancey * distancey); // 2D
            // if (dist2 < 400) { // look ahead feature
            double at2v  = atan2(-e.x, -e.y); // was vx, vy  --chgloor 2012-01-15
            double at2d  = atan2(-distancex, -distancey);
            double at2v2 = atan2(-(*iter)->v.x, -(*iter)->v.y);
            double pi = 3.14159265;
            double s = at2d - at2v;   if (s > pi) s -= 2*pi;   if (s < -pi) s += 2*pi;
            double vv = at2v - at2v2; if (vv > pi) vv -= 2*pi; if (vv < -pi) vv += 2*pi;
            if ((vv < -2.5) || (vv > 2.5)) { // entgegengesetzte richtung
                if ((s < 0) && (s > -0.3)) { // position vor mir, in meine richtung
                    lookforwardcount--;
                }
                if ((s > 0) && (s < 0.3)) {
                    lookforwardcount++;
                }
            }
            // }
        }
    }

    lf.z = 0; // 2d  --chgloor 2012-01-15
    if (lookforwardcount < 0) {
        lf.x = 0.5f *  e.y; // was vx, vy  --chgloor 2012-01-15
        lf.y = 0.5f * -e.x;
    }
    if (lookforwardcount >  0) {
        lf.x = 0.5f * -e.y;
        lf.y = 0.5f *  e.x;
    }
    return lf;
}



/// myForce() is a method that returns an "empty" force (all components set to 0). 
/// This method can be overridden in order to define own forces. 
/// It is called in move() in addition to the other default forces.
/// \date    2012-02-12
/// \return  Tvector: the calculated force
/// \param   e is a vector defining the direction in which the agent wants to walk to. 
Ped::Tvector Ped::Tagent::myForce(Ped::Tvector e) const {
    Ped::Tvector lf;
    return lf;
}


/// Does the agent dynamics stuff. Calls the methods to calculate the individual forces, adds them
/// to get the total force aggecting the agent. This will then be translated into a velocity difference,
/// which is applied to the agents velocity, and then to its position. 
/// \date    2003-12-29
/// \param   h This tells the simulation how far the agent should proceed (also known as Tau in literature). 1 = 1 unit.
void Ped::Tagent::move(double h) {
    int update = 1/h;
    if (timestep % update == 0) {
        desiredforce = desiredForce();
        neighbors = scene->getNeighbors(p.x, p.y, 20);  // 20 = 0.5m which seems rather too low???
        lookaheadforce = lookaheadForce(desiredforce);
        myforce = myForce(desiredforce);
    }
    if (factorsocialforce > 0) socialforce = socialForce();
    if (factorobstacleforce > 0) obstacleforce = obstacleForce();


    //  sum of all forces --> acceleration
    Ped::Tvector a;
    a.x = factorsocialforce * socialforce.x + factordesiredforce * desiredforce.x + factorobstacleforce * obstacleforce.x + factorlookaheadforce * lookaheadforce.x + myforce.x;
    a.y = factorsocialforce * socialforce.y + factordesiredforce * desiredforce.y + factorobstacleforce * obstacleforce.y + factorlookaheadforce * lookaheadforce.y + myforce.y;
    a.z = factorsocialforce * socialforce.z + factordesiredforce * desiredforce.z + factorobstacleforce * obstacleforce.z + factorlookaheadforce * lookaheadforce.z + myforce.z;

    if ( true || gettype() == 2) {
        // do nothing
    } else {
        // calculate the new velocity based on v0 and the acceleration
        /// \todo Make momentum factor (0.75) settable by the user
        v.x = 0.75 * v.x + a.x;
        v.y = 0.75 * v.y + a.y; /// \note Is the momentum factor (0.75) dependent of h?? think so   --chgloor 2012-01-15
        v.z = 0.75 * v.z + a.z;

        double currvmax = vmax;
        double speed = (sqrt(v.x*v.x + v.y*v.y + v.z*v.z));

        if (speed > currvmax) {
            v.x = (v.x / speed) * currvmax;
            v.y = (v.y / speed) * currvmax;
            v.z = (v.z / speed) * currvmax;
        }

        // internal position update == actual move
        p.x = p.x + h * v.x; // x = x0 + v*t
        p.y = p.y + h * v.y;
        p.z = 0; // p.z + h * v.z; // 2D  --chgloor 2012-01-04

        // notice scene of movement
        scene->moveAgent(this);
    }

    timestep++; // local agent tiemstep since creation
}


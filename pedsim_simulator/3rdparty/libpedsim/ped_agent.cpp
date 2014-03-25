//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_scene.h"
#include "ped_obstacle.h"

#include <cmath>
#include <algorithm>
// #include <random>
#include <iostream>

using namespace std;


// default_random_engine generator;


/// Default Constructor
/// \date    2003-12-29
Ped::Tagent::Tagent()
{
    static int staticid = 0;
    id = staticid++;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    v.x = 0;
    v.y = 0;
    v.z = 0;
    type = ADULT;
    teleop = false;
    hasreacheddestination = true;
    destination = NULL;
    lastdestination = NULL;
    birth_waypoint = NULL;
    death_waypoint = NULL;
    follow = -1;
    mlLookAhead = false;
    scene = NULL;
    vmax = randSpeed();

    factorsocialforce = 2.1;
    factorobstacleforce = 10.0;
    factordesiredforce = 1.0;
    factorlookaheadforce = 1.0;

    obstacleForceSigma = 0.8;

    agentRadius = 0.2;

    relaxationTime = 0.5;
}


/// Destructor
/// \date    2012-02-04
Ped::Tagent::~Tagent()
{
}


/// Assigns a Tscene to the agent. Tagent uses this to iterate over all
/// obstacles and other agents in a scene.  The scene will invoke this function
/// when Tscene::addAgent() is called.
/// \date    2012-01-17
/// \warning Bad things will happen if the agent is not assigned to a scene. But usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene ( Ped::Tscene *s )
{
    scene = s;
}


/// Adds a TWaypoint to an agent's list of waypoints. Twaypoints are stored in a
/// cyclic queue, the one just visited is pushed to the back again. There will be a
/// flag to change this behavior soon.
/// \todo Add a flag to change the waypoint queue behavior of the Tagents.
/// \author  chgloor
/// \date    2012-01-19
void Ped::Tagent::addWaypoint ( Twaypoint *wp )
{

    if ( wp->gettype() == Ped::Twaypoint::TYPE_BIRTH )
    {
        birth_waypoint = wp;
        waypoints.push_back ( wp );
    }

    if ( wp->gettype() == Ped::Twaypoint::TYPE_DEATH )
    {
        death_waypoint = wp;
        waypoints.push_back ( wp );
    }

    if ( wp->gettype() == Ped::Twaypoint::TYPE_QUEUE )
    {
        waypoints.push_front ( wp );
    }

    if ( wp->gettype() == Ped::Twaypoint::TYPE_NORMAL || wp->gettype() == Ped::Twaypoint::TYPE_POINT )
    {
        waypoints.push_back ( wp );
    }
}


bool Ped::Tagent::removeWaypoint ( const Twaypoint *wp )
{
    // unset references
    if ( destination == wp )
        destination = NULL;
    if ( lastdestination == wp )
        lastdestination = NULL;

    // remove waypoint from list of destinations
    bool removed = false;
    for ( int i = waypoints.size(); i > 0; --i )
    {
        Twaypoint* currentWaypoint = waypoints.front();
        waypoints.pop_front();
        if ( currentWaypoint != wp )
        {
            waypoints.push_back ( currentWaypoint );
            removed = true;
        }
    }

    return removed;
}


void Ped::Tagent::clearWaypoints()
{
    // unset references
    destination = NULL;
    lastdestination = NULL;

    // remove all references to the waypoints
    // note: don't delete waypoints, because the scene is responsible
	// TODO - remove C++11 fetures and use boost
    for ( int i = waypoints.size(); i > 0; --i )
        waypoints.pop_front();
}


void Ped::Tagent::removeAgentFromNeighbors ( const Ped::Tagent* agentIn )
{
    // search agent in neighbors, and remove him
    set<const Ped::Tagent*>::iterator foundNeighbor = neighbors.find ( agentIn );
    if ( foundNeighbor != neighbors.end() )
        neighbors.erase ( foundNeighbor );
}


/// Sets the agent ID this agent has to follow. If set, the agent will ignore
/// its assigned waypoints and just follow the other agent.
/// \date    2012-01-08
/// \param   id is the agent to follow (must exist, obviously)
/// \todo    Add a method that takes a Tagent* as argument
void Ped::Tagent::setFollow ( int id )
{
    follow = id;
}


/// Gets the ID of the agent this agent is following.
/// \date    2012-01-18
/// \return  int, the agent id of the agent
/// \todo    Add a method that returns a Tagent*
int Ped::Tagent::getFollow() const
{
    return follow;
}


/// Sets the maximum velocity of an agent (vmax). Even if pushed by other
/// agents, it will not move faster than this.
/// \date    2012-01-08
/// \param pvmax The maximum velocity. In scene units per timestep, multiplied by the simulation's precision h.
void Ped::Tagent::setVmax ( double pvmax )
{
    vmax = pvmax;
}


/// Sets the agent's position. This, and other getters returning coordinates,
/// will eventually changed to returning a Tvector.
/// \date    2004-02-10
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition ( double px, double py, double pz )
{
    p.x = px;
    p.y = py;
    p.z = pz;
}


/// Sets the factor by which the social force is multiplied. Values between 0
/// and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorsocialforce ( double f )
{
    factorsocialforce = f;
}

/// Sets the factor by which the obstacle force is multiplied. Values between 0
/// and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorobstacleforce ( double f )
{
    factorobstacleforce = f;
}

/// Sets the factor by which the desired force is multiplied. Values between 0
/// and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactordesiredforce ( double f )
{
    factordesiredforce = f;
}

/// Sets the factor by which the look ahead force is multiplied. Values between
/// 0 and about 10 do make sense.
/// \date    2012-01-20
/// \param   f The factor
void Ped::Tagent::setfactorlookaheadforce ( double f )
{
    factorlookaheadforce = f;
}

/// Set an agent to be stationary/immobile
/// useful for modelling contexts like queues and people standing
void Ped::Tagent::setStationary()
{
    vmax = 0.0;
}


/// Set an agent to be mobile
/// Restoring default social force behaviour
void Ped::Tagent::setMobile()
{
	// TODO - cache the original speed and restore it
    vmax = randSpeed();
}

/// Calculates the force between this agent and the next assigned waypoint.  If
/// the waypoint has been reached, the next waypoint in the list will be
/// selected.  At the moment, a visited waypoint is pushed back to the end of
/// the list, which means that the agents will visit all the waypoints over and
/// over again.  In a later release, this behavior can be controlled by a flag.
/// \date    2012-01-17
/// \todo    move this destination handling into a separate method called by move(). then mark this method as const
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce()
{
    // following behavior
    if ( follow >= 0 )
    {
        Tagent* followedAgent = scene->agents.at ( follow );
        Twaypoint newDestination ( followedAgent->getx(), followedAgent->gety(), 0 );
        newDestination.settype ( Ped::Twaypoint::TYPE_POINT );
        Ped::Tvector ef = newDestination.getForce ( p.x, p.y, 0, 0 );
        desiredDirection = Ped::Tvector ( followedAgent->getx(), followedAgent->gety() );

        // walk with full speed if nothing else affects me
        return vmax * ef;
    }

    // waypoint management (fetch new destination if available)
    // consider: replace hasreacheddestination with destination==NULL
    if ( ( hasreacheddestination == true ) && ( !waypoints.empty() ) )
    {
        destination = waypoints.front();
        // round queue
        waypoints.pop_front();
        waypoints.push_back ( destination );
        hasreacheddestination = false;
    }

    // if there is no destination, don't move
    if ( destination == NULL )
    {
        desiredDirection = Ped::Tvector();
        Tvector antiMove = -v / relaxationTime;
        return antiMove;
    }

    bool reached;
    if ( lastdestination == NULL )
    {
        // create a temporary destination of type point, since no normal from last dest is available
        Twaypoint tempDestination ( destination->getx(), destination->gety(), destination->getr() );
        tempDestination.settype ( Ped::Twaypoint::TYPE_POINT );

        desiredDirection = tempDestination.getForce ( p.x, p.y, 0, 0, &reached );
    }
    else
    {
        desiredDirection = destination->getForce ( p.x, p.y, lastdestination->getx(), lastdestination->gety(), &reached
);
    }


    // mark destination as reached for next time step
    if ( ( hasreacheddestination == false ) && ( reached == true ) )
    {
        hasreacheddestination = true;
        lastdestination = destination;
        destination = NULL;
    }

    // compute force
    Tvector force = desiredDirection.normalized() * vmax;

    return force;
}

/// Calculates the social force between this agent and all the other agents
/// belonging to the same scene.  It iterates over all agents inside the scene,
/// has therefore the complexity O(N^2). A better agent storing structure in
/// Tscene would fix this. But for small (less than 10000 agents) scenarios,
/// this is just fine.
/// \date    2012-01-17
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce() const
{
    // define relative importance of position vs velocity vector
    // (set according to Moussaid-Helbing 2009)
    const double lambdaImportance = 2.0;

    // define speed interaction
    // (set according to Moussaid-Helbing 2009)
    const double gamma = 0.35;

    // define speed interaction
    // (set according to Moussaid-Helbing 2009)
    const double n = 2;

    // define angular interaction
    // (set according to Moussaid-Helbing 2009)
    const double n_prime = 3;

    Tvector force;
    for ( const Ped::Tagent* other: neighbors )
    {
        // don't compute social force to yourself
        if ( other->id == id ) continue;

        // compute difference between both agents' positions
        Tvector diff = other->p - p;

        // skip futher computation if they are too far away from each
        // other. Should speed up things.
        if ( diff.lengthSquared() > 4.0 ) continue;

        Tvector diffDirection = diff.normalized();

        // compute difference between both agents' velocity vectors
        // Note: the agent-other-order changed here
        Tvector velDiff = v - other->v;

        // compute interaction direction t_ij
        Tvector interactionVector = lambdaImportance * velDiff + diffDirection;
        double interactionLength = interactionVector.length();
        Tvector interactionDirection = interactionVector / interactionLength;

        // compute angle theta (between interaction and position difference vector)
        double theta = interactionDirection.angleTo ( diffDirection );
        int thetaSign = ( theta == 0 ) ? ( 0 ) : ( theta/abs ( theta ) );

        // compute model parameter B = gamma * ||D||
        double B = gamma * interactionLength;

        double forceVelocityAmount = -exp ( -diff.length() /B - ( n_prime*B*theta ) * ( n_prime*B*theta ) );
        double forceAngleAmount = -thetaSign * exp ( -diff.length() /B - ( n*B*theta ) * ( n*B*theta ) );

        Tvector forceVelocity = forceVelocityAmount * interactionDirection;
        Tvector forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();

        force += forceVelocity + forceAngle;
    }

    return force;
}


/// Calculates the force between this agent and the nearest obstacle in this scene.
/// Iterates over all obstacles == O(N).
/// \date    2012-01-17
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::obstacleForce() const
{
    // obstacle which is closest only
    Ped::Tvector minDiff;
    double minDistanceSquared = INFINITY;

    for ( const Tobstacle* obstacle : scene->obstacles )
    {
        Ped::Tvector closestPoint = obstacle->closestPoint ( p );
        Ped::Tvector diff = p - closestPoint;
        double distanceSquared = diff.lengthSquared();  // use squared distance to avoid computing square root
        if ( distanceSquared < minDistanceSquared )
        {
            minDistanceSquared = distanceSquared;
            minDiff = diff;
        }
    }

    double distance = sqrt ( minDistanceSquared ) - agentRadius;
    double forceAmount = exp ( -distance/obstacleForceSigma );
    return forceAmount * minDiff.normalized();
}


/// Calculates the mental layer force of the strategy "look ahead". It is
/// implemented here in the physical layer because of performance reasons. It
/// iterates over all Tagents in the Tscene, complexity O(N^2).
/// \date    2012-01-17
/// \return  Tvector: the calculated force
/// \param e is a vector defining the direction in which the agent should look
///          ahead to. Usually, this is the direction he wants to walk to.
Ped::Tvector Ped::Tagent::lookaheadForce ( Ped::Tvector e ) const
{
    const double pi = 3.14159265;
    int lookforwardcount = 0;
    for ( set<const Ped::Tagent*>::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter )
    {
        const Ped::Tagent* other = *iter;

        // don't compute this force for the agent himself
        if ( other->id == id )
            continue;

        double distancex = other->p.x - p.x;
        double distancey = other->p.y - p.y;
        double at2v = atan2 ( -e.x, -e.y ); // was vx, vy  --chgloor 2012-01-15
        double at2d = atan2 ( -distancex, -distancey );
        double at2v2 = atan2 ( -other->v.x, -other->v.y );
        double s = at2d - at2v;
		
        if ( s > pi ) s -= 2*pi;
        if ( s < -pi ) s += 2*pi;
        double vv = at2v - at2v2;
        if ( vv > pi ) vv -= 2*pi;
        if ( vv < -pi ) vv += 2*pi;
        if ( abs ( vv ) > 2.5 ) // opposite direction
        {
            if ( ( s < 0 ) && ( s > -0.3 ) ) // position vor mir, in meine richtung
                lookforwardcount--;
            if ( ( s > 0 ) && ( s < 0.3 ) )
                lookforwardcount++;
        }
    }

    Ped::Tvector lf;
    if ( lookforwardcount < 0 )
    {
        lf.x = 0.5f *  e.y; // was vx, vy  --chgloor 2012-01-15
        lf.y = 0.5f * -e.x;
    }
    if ( lookforwardcount > 0 )
    {
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
Ped::Tvector Ped::Tagent::myForce ( Ped::Tvector e )
{
    Ped::Tvector lf;
    return lf;
}


void Ped::Tagent::computeForces()
{
    const double neighborhoodRange = 20.0;

    desiredforce = desiredForce();
    neighbors = scene->getNeighbors ( p.x, p.y, neighborhoodRange );
    if ( factorlookaheadforce > 0 ) lookaheadforce = lookaheadForce ( desiredDirection );
    if ( factorsocialforce > 0 ) socialforce = socialForce();
    if ( factorobstacleforce > 0 ) obstacleforce = obstacleForce();
    myforce = myForce ( desiredDirection );
}


/// Does the agent dynamics stuff. Calls the methods to calculate the individual forces, adds them
/// to get the total force affecting the agent. This will then be translated into a velocity difference,
/// which is applied to the agents velocity, and then to its position.
/// \date    2003-12-29
/// \param   stepSizeIn This tells the simulation how far the agent should proceed
void Ped::Tagent::move ( double stepSizeIn )
{
    // sum of all forces --> acceleration
    a = factordesiredforce * desiredforce
        + factorsocialforce * socialforce
        + factorobstacleforce * obstacleforce
        + factorlookaheadforce * lookaheadforce
        + myforce;

    v = v + stepSizeIn * a;

    // don't exceed maximal speed
    if ( v.length() > vmax ) v = v.normalized() * vmax;

    // internal position update = actual move
    p = p + stepSizeIn * v;

    // notice scene of movement
    scene->moveAgent ( this );
}

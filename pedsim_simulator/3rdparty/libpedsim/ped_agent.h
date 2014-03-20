//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#ifndef _ped_agent_h_
#define _ped_agent_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include "ped_vector.h"

#include <deque>
#include <set>
#include <vector>


#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>

using namespace std;


static boost::mt19937 rng; 


inline double randSpeed()
{
    // rng.seed(static_cast<unsigned int>(std::time(0)));
    boost::normal_distribution<> nd(1.34, 0.26);

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);

    double d = var_nor();

    return d;
}


namespace Ped {
    class Tscene;
    class Twaypoint;

/// \example example.cpp


/// This is the main class of the library. It contains the Tagent, which eventually will move through the
/// Tscene and interact with Tobstacle and other Tagent. You can use it as it is, and access the agent's
/// coordinates using the getx() etc methods. Or, if you want to change the way the agent behaves, you can
/// derive a new class from it, and overwrite the methods you want to change. This is also a convenient way
/// to get access to internal variables not available though public methods, like the individual forces that
/// affect the agent.
/// \author  chgloor
/// \date    2003-12-26
    class LIBEXPORT Tagent {
    public:
        enum AgentType
        {
            ADULT = 0,
            CHILD = 1,
            ROBOT = 2,
            ELDERLY = 3,
			STANDING = 4	// to simulate groups of people standing
        };

    public:
        Tagent();
        virtual ~Tagent();

        virtual void computeForces();
        virtual void move(double stepSizeIn);
        virtual Tvector desiredForce();
        virtual Tvector socialForce() const;
        virtual Tvector obstacleForce() const;
        virtual Tvector lookaheadForce(Tvector desired) const;
        virtual Tvector myForce(Tvector desired);
		virtual void updateState(int event = 0) {}

        void setPosition(double px, double py, double pz);
        void setType(AgentType t) { this->type = t; };
        void setFollow(int id);
        void setVmax(double vmax);
        void setvx(const double vx) {v.x = vx;}
        void setvy(const double vy) {v.y = vy;}
        void setteleop(const bool val) { teleop = val; }

        int getFollow() const;

        int getid() const { return id; };
        AgentType gettype() const { return type; };
        double getvmax() const { return vmax; };
        bool getteleop() { return teleop; }
        double getRadius() { return agentRadius; }

        // these getter should replace the ones later (returning the individual vector values)
        const Tvector& getPosition() const { return p; }
        const Tvector& getVelocity() const { return v; }
        const Tvector& getAcceleration() const { return a; }

        double getx() const { return p.x; };
        double gety() const { return p.y; };
        double getz() const { return p.z; };
        double getax() const { return a.x; };
        double getay() const { return a.y; };
        double getaz() const { return a.z; };
        double getvx() const { return v.x; };
        double getvy() const { return v.y; };
        double getvz() const { return v.z; };

        Twaypoint* getBirthWaypoint() { return birth_waypoint; }
        Twaypoint* getDeathWaypoint() { return death_waypoint; }
        Twaypoint* getDestination() { return destination; }

        void setfactorsocialforce(double f);
        void setfactorobstacleforce(double f);
        void setfactordesiredforce(double f);
        void setfactorlookaheadforce(double f);
        
        void setStationary();
        void setMobile();

        void assignScene(Tscene* s);
        void addWaypoint(Twaypoint* wp);
        bool removeWaypoint(const Twaypoint* wp);
        void clearWaypoints();
        void removeAgentFromNeighbors(const Tagent* agentIn);

    protected:
        int id;                                           ///< agent number
        Tvector p;                                        ///< current position of the agent
        Tvector v;                                        ///< velocity of the agent
        Tvector a;                                        ///< current acceleration of the agent
        AgentType type;
        double vmax;                                      ///< individual max velocity per agent
        int follow;
        bool teleop;

        Ped::Tvector desiredDirection;

        Ped::Tscene* scene;

        deque<Twaypoint*> waypoints;                      ///< coordinates of the next destinations
        Twaypoint* destination;                           ///< coordinates of the next destination
        Twaypoint* lastdestination;                       ///< coordinates of the last destination
        bool hasreacheddestination;                       ///< true if it has reached its destination

        Twaypoint *birth_waypoint;          // used for flow generation
        Twaypoint *death_waypoint;

        bool mlLookAhead;

        double factordesiredforce;
        double factorsocialforce;
        double factorobstacleforce;
        double factorlookaheadforce;

        double obstacleForceSigma;

        Ped::Tvector desiredforce;
        Ped::Tvector socialforce;
        Ped::Tvector obstacleforce;
        Ped::Tvector lookaheadforce;
        Ped::Tvector myforce;

        double relaxationTime;

        double agentRadius;

        set<const Ped::Tagent*> neighbors;

        long timestep;
    };
}
#endif

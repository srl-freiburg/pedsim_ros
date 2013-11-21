//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#include "math.h"

#include "ped_waypoint.h"
#include <iostream>

#include <vector>

using namespace std;

const int Ped::Twaypoint::TYPE_NORMAL = 0;
const int Ped::Twaypoint::TYPE_POINT  = 1;
const int Ped::Twaypoint::TYPE_BIRTH  = 2;
const int Ped::Twaypoint::TYPE_DEATH  = 3;


/// Constructor: Sets some intial values. The agent has to pass within the given radius. 
/// \date    2012-01-07
/// \param   px The x coordinate of the waypoint
/// \param   py The y coordinate of the waypoint
/// \param   pr The radius of the waypoint
Ped::Twaypoint::Twaypoint(double px, double py, double pr) : x(px), y(py), r(pr), type(Ped::Twaypoint::TYPE_NORMAL) {};


/// Constructor - sets the most basic parameters.
/// \date    2012-01-07
Ped::Twaypoint::Twaypoint() : x(0), y(0), r(1), type(Ped::Twaypoint::TYPE_NORMAL) {};


/// Default Destructor
/// \author  chgloor
/// \date    2012-02-04
Ped::Twaypoint::~Twaypoint() {};


/// Calculates the point that is on the given line and normal to the given position.
/// If it is not inside the line, the start or end point of the line is returned.
/// \date    2012-01-10
/// \param   p1 The x coordinate of the point outside the obstacle
/// \param   p2 The y coordinate of the point outside the obstacle
/// \param   oc11 The x coordinate of the first corner of the obstacle
/// \param   oc12 The y coordinate of the first corner of the obstacle
/// \param   oc21 The x coordinate of the second corner of the obstacle
/// \param   oc22 The y coordinate of the second corner of the obstacle
/// \return  Tvector The calculated point
Ped::Tvector Ped::Twaypoint::normalpoint(double p1, double p2, double oc11, double oc12, double oc21, double oc22) const {
    double a1 = oc11;
    double a2 = oc12;
    double b1 = oc21 - oc11;
    double b2 = oc22 - oc12;

    double lambda = (p1*b1 + p2*b2 - b1*a1 - b2*a2) / (b1*b1 + b2*b2);

    if (lambda <= 0) { return Ped::Tvector(oc11, oc12, 0); };
    if (lambda >= 1) { return Ped::Tvector(oc21, oc22, 0); };

    return Ped::Tvector(a1 + lambda*b1, a2 + lambda*b2,  0);
} 


/// Returns the force into the direction of the waypoint
/// \date    2012-01-10
/// \param   myx The x coordinate of the current position of the agent
/// \param   myy The y coordinate of the current position of the agent
/// \param   fromx The x coordinate of the last assigned waypoint, i.e. where the agent is coming from
/// \param   fromy The y coordinate of the last assigned waypoint, i.e. where the agent is coming from
/// \param   *reached Set to true if the agent has reached the waypoint in this call.
/// \return  Tvector The calculated force
Ped::Tvector Ped::Twaypoint::getForce(double myx, double myy, double fromx, double fromy, bool *reached) const {
    Ped::Tvector f;

    if ( type == Ped::Twaypoint::TYPE_NORMAL || 
        type == Ped::Twaypoint::TYPE_BIRTH || 
        type == Ped::Twaypoint::TYPE_DEATH) 
    {
        double distancex = x - fromx;
        double distancey = y - fromy;
        double dist2 = (distancex * distancex + distancey * distancey);  // dist2 = distanz im quadrat
        double dist = sqrt(dist2);
        
        if ( dist <= 0 ) {
          f.x = 0;
          f.y = 0;
          return f;
        }

        double normalex = distancey / dist;
        double normaley = distancex / dist;

        double oc11 = x + r * normalex;
        double oc12 = y - r * normaley;
        double oc21 = x - r * normalex;
        double oc22 = y + r * normaley;

        Ped::Tvector pnormal = normalpoint(myx, myy, oc11, oc12, oc21, oc22);

        double pndistancex = myx - pnormal.x;
        double pndistancey = myy - pnormal.y;
        double pndist2 = (pndistancex * pndistancex + pndistancey * pndistancey);  // dist2 = distanz im quadrat
        double pndist = sqrt(pndist2);

        if (pndist < 1) { *reached = true; } else { *reached = false; }
        if (pndist == 0) { f.x = 0; f.y = 0; return f; }
        f.x = -pndistancex / pndist;
        f.y = -pndistancey / pndist;

    } else if (type == Ped::Twaypoint::TYPE_POINT) {
        double distancex = x - myx;
        double distancey = y - myy;
        double dist2 = (distancex * distancex + distancey * distancey);  // dist2 = distanz im quadrat
        double dist = sqrt(dist2);

        if ( dist <= 0 ) {
          f.x = 0;
          f.y = 0;
          return f;
        }

        f.x = distancex / dist;
        f.y = distancey / dist;
        if (dist < r) { *reached = true; } else { *reached = false; };
    }

    return f;
}

//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#include "ped_waypoint.h"

#include <cmath>

// initialize static variables
int Ped::Twaypoint::staticid = 0;


/// Constructor: Sets some intial values. The agent has to pass within the given radius.
/// \date    2012-01-07
/// \param   px The x coordinate of the waypoint
/// \param   py The y coordinate of the waypoint
/// \param   pr The radius of the waypoint
Ped::Twaypoint::Twaypoint(double px, double py, double pr) : id(staticid++), x(px), y(py), r(pr), type(Ped::Twaypoint::TYPE_NORMAL) {};


/// Constructor - sets the most basic parameters.
/// \date    2012-01-07
Ped::Twaypoint::Twaypoint() : id(staticid++), x(0), y(0), r(1), type(Ped::Twaypoint::TYPE_NORMAL) {};


/// Default Destructor
/// \author  chgloor
/// \date    2012-02-04
Ped::Twaypoint::~Twaypoint() {};


/// Calculates the point that is on the given line and normal to the given position.
/// If it is not inside the line, the start or end point of the line is returned.
/// \date    2012-01-10
/// \param   p The point outside the obstacle
/// \param   normalLineStart The first corner of the normal line
/// \param   normalLineEnd The second corner of the normal line
/// \return  Tvector The calculated point
Ped::Tvector Ped::Twaypoint::normalpoint(const Ped::Tvector& p, const Ped::Tvector& normalLineStart, const Ped::Tvector& normalLineEnd) const {
    Ped::Tvector relativeEnd = normalLineEnd - normalLineStart;

    double lambda = (Tvector::dotProduct(p, relativeEnd) - Tvector::dotProduct(normalLineStart, relativeEnd)) / relativeEnd.lengthSquared();

    if (lambda <= 0)
        return normalLineStart;
    else if (lambda >= 1)
        return normalLineEnd;
    else
        return normalLineStart + lambda*relativeEnd;
}


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
    return normalpoint(Tvector(p1, p2), Tvector(oc11, oc12), Tvector(oc21, oc22));
}


/// Returns the force into the direction of the waypoint
/// \date    2012-01-10
/// \param   agentX The x coordinate of the current position of the agent
/// \param   agentY The y coordinate of the current position of the agent
/// \param   fromx The x coordinate of the last assigned waypoint, i.e. where the agent is coming from
/// \param   fromy The y coordinate of the last assigned waypoint, i.e. where the agent is coming from
/// \param   *reached Set to true if the agent has reached the waypoint in this call.
/// \return  Tvector The calculated force
Ped::Tvector Ped::Twaypoint::getForce(double agentX, double agentY, double fromx, double fromy, bool *reached) const {
    if(type == Ped::Twaypoint::TYPE_NORMAL) {
        Tvector diff(x - agentX, y - agentY);

        if(reached != NULL) {
            if(diff.length() < r)
                *reached = true;
            else
                *reached = false;
        }
        return diff.normalized();
    }
    else if(type == Ped::Twaypoint::TYPE_POINT) {
        Tvector diff(x - agentX, y - agentY);

        if(reached != NULL) {
            if(diff.length() < r)
                *reached = true;
            else
                *reached = false;
        }
        return diff.normalized();
    }
    else {
        // unknown waypoint type
        return Tvector();
    }
}

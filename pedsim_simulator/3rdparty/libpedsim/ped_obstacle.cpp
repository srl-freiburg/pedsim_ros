//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#include "math.h"

#include "ped_obstacle.h"

#include <vector>

using namespace std;

/// Constructor used to set intial values.
/// \date    2012-01-07
/// \param pax x coordinate of the first corner of the obstacle.
/// \param pay y coordinate of the first corner of the obstacle.
/// \param pbx x coordinate of the second corner of the obstacle.
/// \param pby y coordinate of the second corner of the obstacle.
Ped::Tobstacle::Tobstacle(double pax, double pay, double pbx, double pby) {
    static int staticid = 0;
    id = staticid++;
    ax = pax;
    ay = pay;
    bx = pbx;
    by = pby;
};


/// Default constructor, places a wall from 0/0 to 1/1
/// \date    2012-01-07
Ped::Tobstacle::Tobstacle() {
    static int staticid = 0;
    id = staticid++;
    ax = 0;
    ay = 0;
    bx = 1;
    by = 1;
};


/// Moves the obstacle to a new position. Can be uses to simulate opening doors etc. 
/// \date    2012-01-07
/// \param pax x coordinate of the first corner of the obstacle.
/// \param pay y coordinate of the first corner of the obstacle.
/// \param pbx x coordinate of the second corner of the obstacle.
/// \param pby y coordinate of the second corner of the obstacle.
void Ped::Tobstacle::setPosition(double pax, double pay, double pbx, double pby) {
    ax = pax, ay = pay;
    bx = pbx, by = pby;
};


/// Calculates and returns the forces of the obstacle to a given point x/y. 
/// x/y can be the location of an agent, but it can also be anything else, 
/// for example a grid coordinate of the user interface, if you want to display
/// the obstacle forces on the map.
/// \date    2012-01-17
/// \return  Tvector forces
/// \param   double x: The x coordinate of the point
/// \param   double y: The y coordinate of the point
Ped::Tvector Ped::Tobstacle::obstacleforce(double x, double y) {	
    double a1 = this->ax;
    double a2 = this->ay;
    double b1 = this->bx - this->ax;
    double b2 = this->by - this->ay;
    double lambda = (x*b1 + y*b2 - b1*a1 - b2*a2) / (b1*b1 + b2*b2);

    Ped::Tvector v; v.z = 0;
    if (lambda <= 0) { v.x = ax; v.y = ay; return v; };
    if (lambda >= 1) { v.x = bx; v.y = by; return v; };

    v.x = a1 + lambda*b1;
    v.y = a2 + lambda*b2;
    return v;
}

/// rot phi around x/y
/// \author  chgloor
/// \date    2012-01-20
/// \warning Due to rounding errors, this will fail after a while. 
/// \todo    Use the original points (saved) and cache the total phi or something.
/// \param   x The x coordinate of the point the obstacle will be rotated around.
/// \param   y The y coordinate of the point the obstacle will be rotated around.
/// \param   r The angle the obstacle will be rotated, where phi is given in radians
void Ped::Tobstacle::rotate(double x, double y, double phi) {
    double anx = getax()*cos(phi) - x*cos(phi) - getay()*sin(phi) + y*sin(phi) + x;
    double any = getax()*sin(phi) - x*sin(phi) + getay()*cos(phi) - y*cos(phi) + y;

    double bnx = getbx()*cos(phi) - x*cos(phi) - getby()*sin(phi) + y*sin(phi) + x;
    double bny = getbx()*sin(phi) - x*sin(phi) + getby()*cos(phi) - y*cos(phi) + y;

    setPosition(anx, any, bnx, bny);
}

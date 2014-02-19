//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#ifndef _ped_obstacle_h_
#define _ped_obstacle_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include "ped_vector.h"

using namespace std;

namespace Ped {

/// Class that defines a Tobstacle object. An obstacle is, for now, always a wall with start and end coordinate.
/// \author  chgloor
/// \date    2012-01-17
class LIBEXPORT Tobstacle {
private:
    int id;                                            ///< Obstacle number
    double ax;                                         ///< Position of the obstacle
    double ay;                                         ///< Position of the obstacle
    double bx;                                         ///< Position of the obstacle
    double by;                                         ///< Position of the obstacle
    int type;

public:
    Tobstacle();
    Tobstacle(double ax, double ay, double bx, double by);

    virtual void setPosition(double ax, double ay, double bx, double by);
    virtual Tvector obstacleforce(double p1, double p2);
    virtual void rotate(double x, double y, double phi);

    void setType(int t) {type = t; };
    int getid() { return id; };
    int gettype() { return type; };
    double getax() { return ax; };
    double getay() { return ay; };
    double getbx() { return bx; };
    double getby() { return by; };
};

}


#endif

//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#ifndef _ped_waypoint_h_
#define _ped_waypoint_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include "ped_vector.h"

using namespace std;

namespace Ped {

/// The waypoint classs
/// \author  chgloor
/// \date    2012-01-07
class LIBEXPORT Twaypoint {

public:
    Twaypoint();
    Twaypoint(double x, double y, double r);
    virtual ~Twaypoint();

    virtual Tvector getForce(double myx, double myy, double fromx, double fromy, bool *reached) const;
    virtual Tvector normalpoint(double p1, double p2, double oc11, double oc12, double oc21, double oc22) const;

    void setType(int t) {type = t; };
    void setx(double px) { x = px; };
    void sety(double py) { y = py; };
    void setr(double pr) { r = pr; };
    void settype(int t) { type = t; };

    int getid() const { return id; };
    int gettype() const { return type; };
    double getx() const { return x; };
    double gety() const { return y; };
    double getr() const { return r; };

    static const int TYPE_NORMAL;
    static const int TYPE_POINT;
    static const int TYPE_BIRTH;    // new agents are born here
    static const int TYPE_DEATH;    // agents die here

private:
    int id;                                           ///< waypoint number
    double x;                                         ///< position of the waypoint
    double y;                                         ///< position of the waypoint
    double r;                                         ///< position of the waypoint
    int type;                                         ///< type of the waypoint (Ped::Twaypoint::TYPE_xxx)

};

};

#endif

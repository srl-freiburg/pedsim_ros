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

#include <cstddef>
#include "ped_vector.h"

using namespace std;

namespace Ped {
// Forward Declarations
class Tagent;

/// The waypoint class
/// \author  chgloor
class LIBEXPORT Twaypoint {
 public:
  enum WaypointType { AreaWaypoint = 0, PointWaypoint = 1 };
  enum Behavior { SIMPLE = 0, SOURCE = 1, SINK = 2 };

 public:
  Twaypoint(double xIn = 0, double yIn = 0);
  Twaypoint(const Tvector& posIn);
  virtual ~Twaypoint();

  int getId() const { return id; };
  int getType() const { return type; };
  int getBehavior() const { return behavior; };
  Ped::Tvector getPosition() const { return position; };
  double getx() const { return position.x; };
  double gety() const { return position.y; };
  double getRadius() const { return radius; };

  virtual void setPosition(double xIn, double yIn) {
    position.x = xIn;
    position.y = yIn;
  };
  virtual void setPosition(const Ped::Tvector& positionIn) {
    position = positionIn;
  };
  virtual void setx(double xIn) { position.x = xIn; };
  virtual void sety(double yIn) { position.y = yIn; };
  virtual void setRadius(double rIn) { radius = rIn; };
  void setType(WaypointType t) { type = t; };
  void setBehavior(Behavior b) { behavior = b; };

  virtual Tvector getForce(const Tagent& agent,
                           Ped::Tvector* desiredDirectionOut = NULL,
                           bool* reached = NULL) const;
  virtual Tvector closestPoint(const Tvector& p,
                               bool* withinWaypoint = NULL) const;

 protected:
  static int staticid;                   ///< last waypoint number
  int id;                                ///< waypoint number
  Tvector position;                      ///< position of the waypoint
  WaypointType type;                     ///< type of the waypoint
  Behavior behavior = Behavior::SIMPLE;  ///< behavior of the waypoint
  double radius;                          ///< radius of the waypoint
};
}

#endif

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

namespace Ped {

/// Class that defines a Tobstacle object. An obstacle is, for now, always a
/// wall with start and end coordinate.
/// \author  chgloor
/// \date    2012-01-17
class LIBEXPORT Tobstacle {
 public:
  Tobstacle();
  Tobstacle(double ax, double ay, double bx, double by);
  Tobstacle(const Tvector& startIn, const Tvector& endIn);
  virtual ~Tobstacle();

  int getid() const { return id; };
  int gettype() const { return type; };
  double getax() const { return ax; };
  double getay() const { return ay; };
  double getbx() const { return bx; };
  double getby() const { return by; };
  Tvector getStartPoint() const;
  Tvector getEndPoint() const;

  virtual void setPosition(double ax, double ay, double bx, double by);
  virtual void setPosition(const Tvector& startIn, const Tvector& endIn);
  virtual void setStartPoint(const Tvector& startIn);
  virtual void setEndPoint(const Tvector& endIn);
  virtual void setType(int t) { type = t; };

  virtual Tvector closestPoint(double p1, double p2) const;
  virtual Tvector closestPoint(const Tvector& pointIn) const;
  virtual void rotate(const Tvector& rotationCenterIn,
                      const Ped::Tangle& angleIn);

 protected:
  static int staticid;
  int id;     ///< Obstacle number
  double ax;  ///< Position of the obstacle
  double ay;  ///< Position of the obstacle
  double bx;  ///< Position of the obstacle
  double by;  ///< Position of the obstacle
  int type;
};
}

#endif

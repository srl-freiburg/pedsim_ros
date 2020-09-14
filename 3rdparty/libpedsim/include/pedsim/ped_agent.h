//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
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

using namespace std;

namespace Ped {
class Tscene;
class Twaypoint;

/// \example example.cpp

/// This is the main class of the library. It contains the Tagent, which
/// eventually will move through the
/// Tscene and interact with Tobstacle and other Tagent. You can use it as it
/// is, and access the agent's
/// coordinates using the getx() etc methods. Or, if you want to change the way
/// the agent behaves, you can
/// derive a new class from it, and overwrite the methods you want to change.
/// This is also a convenient way
/// to get access to internal variables not available though public methods,
/// like the individual forces that
/// affect the agent.
/// \author  chgloor
/// \date    2003-12-26
class LIBEXPORT Tagent {
 public:
  enum AgentType { ADULT = 0, CHILD = 1, ROBOT = 2, ELDER = 3 };

  Tagent();
  virtual ~Tagent();

  virtual void updateState(){};
  virtual void computeForces();
  virtual void move(double stepSizeIn);
  virtual Tvector desiredForce();
  virtual Tvector socialForce() const;
  virtual Tvector obstacleForce() const;
  virtual Tvector myForce(Tvector desired) const;
  virtual Twaypoint* getCurrentWaypoint() const = 0;

  virtual void setPosition(double px, double py, double pz = 0);
  virtual void setType(AgentType typeIn) { type = typeIn; };
  virtual void setVmax(double vmax);
  virtual void SetRadius(double radius) { agentRadius = radius; }

  void setTeleop(bool opstatus) { teleop = opstatus; }
  void setRobotPosDiffScalingFactor(double scalingFactor);

  int getId() const { return id; };
  AgentType getType() const { return type; };
  double getVmax() const { return vmax; };
  double getRelaxationTime() const { return relaxationTime; };
  bool getTeleop() { return teleop; }
  double getRobotPosDiffScalingFactor() const { return robotPosDiffScalingFactor; };

  // these getter should replace the ones later (returning the individual vector
  // values)
  const Tvector& getPosition() const { return p; }
  const Tvector& getVelocity() const { return v; }
  const Tvector& getAcceleration() const { return a; }

  double getx() const { return p.x; };
  double gety() const { return p.y; };
  double getz() const { return p.z; };
  double getvx() const { return v.x; };
  double getvy() const { return v.y; };
  double getvz() const { return v.z; };
  double getax() const { return a.x; };
  double getay() const { return a.y; };
  double getaz() const { return a.z; };

  void setvx(double vv) { v.x = vv; }
  void setvy(double vv) { v.y = vv; }

  virtual void setForceFactorDesired(double f);
  virtual void setForceFactorSocial(double f);
  virtual void setForceFactorObstacle(double f);

  void assignScene(Tscene* sceneIn);
  void removeAgentFromNeighbors(const Tagent* agentIn);

 protected:
  int id;
  Tvector p;  ///< current position of the agent
  Tvector v;  ///< current velocity of the agent
  Tvector a;  ///< current acceleration of the agent
  AgentType type;
  double vmax;
  double agentRadius;
  double relaxationTime;
  bool teleop;
  double robotPosDiffScalingFactor;

  double forceFactorDesired;
  double forceFactorSocial;
  double forceFactorObstacle;
  double forceSigmaObstacle;

  Ped::Tscene* scene;

  Ped::Tvector desiredDirection;
  set<const Ped::Tagent*> neighbors;

  Ped::Tvector desiredforce;
  Ped::Tvector socialforce;
  Ped::Tvector obstacleforce;
  Ped::Tvector myforce;
};
}
#endif

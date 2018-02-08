//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#include "ped_obstacle.h"

#include <cmath>
#include <vector>

using namespace std;

int Ped::Tobstacle::staticid = 0;

/// Default constructor, places a wall from 0/0 to 1/1
/// \date    2012-01-07
Ped::Tobstacle::Tobstacle() {
  id = staticid++;
  ax = 0;
  ay = 0;
  bx = 1;
  by = 1;
  type = 0;
}

/// Constructor used to set initial values.
/// \date    2012-01-07
/// \param pax x coordinate of the first corner of the obstacle.
/// \param pay y coordinate of the first corner of the obstacle.
/// \param pbx x coordinate of the second corner of the obstacle.
/// \param pby y coordinate of the second corner of the obstacle.
Ped::Tobstacle::Tobstacle(double pax, double pay, double pbx, double pby) {
  id = staticid++;
  ax = pax;
  ay = pay;
  bx = pbx;
  by = pby;
  type = 0;
}

/// Constructor used to set initial values.
/// \date    2013-08-02
/// \param startIn The first corner of the obstacle.
/// \param endIn The second corner of the obstacle.
Ped::Tobstacle::Tobstacle(const Tvector& startIn, const Tvector& endIn) {
  id = staticid++;
  ax = startIn.x;
  ay = startIn.y;
  bx = endIn.x;
  by = endIn.y;
  type = 0;
}

/// Destructor
Ped::Tobstacle::~Tobstacle() {
  // clean up
}

Ped::Tvector Ped::Tobstacle::getStartPoint() const { return Tvector(ax, ay); }

Ped::Tvector Ped::Tobstacle::getEndPoint() const { return Tvector(bx, by); }

/// Moves the obstacle to a new position. Can be uses to simulate opening doors
/// etc.
/// \date    2012-01-07
/// \param pax x coordinate of the first corner of the obstacle.
/// \param pay y coordinate of the first corner of the obstacle.
/// \param pbx x coordinate of the second corner of the obstacle.
/// \param pby y coordinate of the second corner of the obstacle.
void Ped::Tobstacle::setPosition(double pax, double pay, double pbx,
                                 double pby) {
  ax = pax;
  ay = pay;
  bx = pbx;
  by = pby;
}

void Ped::Tobstacle::setPosition(const Tvector& startIn, const Tvector& endIn) {
  setPosition(startIn.x, startIn.y, endIn.x, endIn.y);
}

void Ped::Tobstacle::setStartPoint(const Tvector& startIn) {
  ax = startIn.x;
  ay = startIn.y;
}

void Ped::Tobstacle::setEndPoint(const Tvector& endIn) {
  bx = endIn.x;
  by = endIn.y;
}

Ped::Tvector Ped::Tobstacle::closestPoint(const Tvector& pointIn) const {
  Tvector startPoint(ax, ay);
  Tvector endPoint(bx, by);
  Tvector relativeEndPoint = endPoint - startPoint;

  Tvector relativePoint = pointIn - startPoint;
  double lambda = (Tvector::dotProduct(relativePoint, relativeEndPoint)) /
                  relativeEndPoint.lengthSquared();

  if (lambda <= 0)
    return startPoint;
  else if (lambda >= 1)
    return endPoint;
  else
    return startPoint + lambda * relativeEndPoint;
}

/// Calculates and returns the forces of the obstacle to a given point x/y.
/// x/y can be the location of an agent, but it can also be anything else,
/// for example a grid coordinate of the user interface, if you want to display
/// the obstacle forces on the map.
/// \date    2012-01-17
/// \return  Tvector forces
/// \param   double x: The x coordinate of the point
/// \param   double y: The y coordinate of the point
Ped::Tvector Ped::Tobstacle::closestPoint(double p1, double p2) const {
  return closestPoint(Tvector(p1, p2));
}

/// Rotate obstacle
/// \author  chgloor
/// \warning Due to rounding errors, this will fail after a while.
/// \param   rotationCenterIn The point the obstacle will be rotated around.
/// \param   angleIn The angle the obstacle will be rotated, where phi is given
/// in radians
void Ped::Tobstacle::rotate(const Ped::Tvector& rotationCenterIn,
                            const Ped::Tangle& angleIn) {
  double angleValue = angleIn.toRadian();
  double sinPhi = sin(angleValue);
  double cosPhi = cos(angleValue);

  Ped::Tvector relativeStart = getStartPoint() - rotationCenterIn;
  Ped::Tvector relativeEnd = getEndPoint() - rotationCenterIn;

  // computation
  // NOTE: currently there is no matrix support in PedSim
  Ped::Tvector rotatedStart =
      Ped::Tvector(cosPhi * relativeStart.x - sinPhi * relativeStart.y,
                   sinPhi * relativeStart.x + cosPhi * relativeStart.y) +
      rotationCenterIn;
  Ped::Tvector rotatedEnd =
      Ped::Tvector(cosPhi * relativeEnd.x - sinPhi * relativeEnd.y,
                   sinPhi * relativeEnd.x + cosPhi * relativeEnd.y) +
      rotationCenterIn;

  // update position
  setPosition(rotatedStart, rotatedEnd);
}

/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <pedsim_simulator/element/areawaypoint.h>

AreaWaypoint::AreaWaypoint(const QString& nameIn,
                           const Ped::Tvector& positionIn, double rIn)
    : Waypoint(nameIn, positionIn) {
  // initialize values
  radius = rIn;
}

AreaWaypoint::AreaWaypoint(const QString& nameIn, double xIn, double yIn,
                           double rIn)
    : Waypoint(nameIn, Ped::Tvector(xIn, yIn)) {
  // initialize values
  radius = rIn;
}

AreaWaypoint::~AreaWaypoint() {}

QString AreaWaypoint::getName() const { return name; }

bool AreaWaypoint::isWithinArea(const Ped::Tvector& posIn) {
  Ped::Tvector diff = getPosition() - posIn;
  double distance = diff.length();

  return (distance <= radius);
}

Ped::Tvector AreaWaypoint::closestPoint(const Ped::Tvector& posIn,
                                        bool* withinWaypoint) const {
  Ped::Tvector diff = position - posIn;

  if (diff.length() <= radius) {
    if (withinWaypoint != NULL) *withinWaypoint = true;

    return posIn;
  } else {
    if (withinWaypoint != NULL) *withinWaypoint = false;

    Ped::Tvector direction = diff.normalized();
    return position + radius * direction;
  }
}

double AreaWaypoint::getRadius() const { 
  return getRadius();
  // return radius; 
}

void AreaWaypoint::setRadius(double rIn) {
  // update radius
  // radius = rIn;
  setRadius(rIn);

  // inform user
  emit radiusChanged(radius);
}

QPointF AreaWaypoint::getVisiblePosition() const {
  return QPointF(getx(), gety());
}

void AreaWaypoint::setVisiblePosition(const QPointF& positionIn) {
  setPosition(positionIn.x(), positionIn.y());
}

QString AreaWaypoint::toString() const {
  return tr("AreaWaypoint '%1' (@%2,%3)").arg(name).arg(getx()).arg(gety());
}

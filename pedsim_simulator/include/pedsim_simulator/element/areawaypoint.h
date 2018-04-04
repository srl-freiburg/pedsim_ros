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

#ifndef _areawaypoint_h_
#define _areawaypoint_h_

#include <pedsim_simulator/element/waypoint.h>
#include <QPointF>

class AreaWaypoint : public Waypoint {
  Q_OBJECT

  // Constructor and Destructor
 public:
  AreaWaypoint(const QString& nameIn, const Ped::Tvector& positionIn,
               double rIn = 1);
  AreaWaypoint(const QString& nameIn, double xIn = 0, double yIn = 0,
               double rIn = 1);

  virtual ~AreaWaypoint();

  // Signals
 signals:
  void radiusChanged(double radius);

  // Methods
 public:
  QString getName() const;

  bool isWithinArea(const Ped::Tvector& posIn);

  virtual double getRadius() const;
  virtual void setRadius(double rIn);

  // → Ped::Twaypoint Overrides
  virtual Ped::Tvector closestPoint(const Ped::Tvector& posIn,
                                    bool* withinWaypoint = NULL) const;

  // → ScenarioElement Overrides/Overloads
 public:
  virtual QPointF getVisiblePosition() const;
  virtual void setVisiblePosition(const QPointF& positionIn);
  QString toString() const;

  // Attributes
 public:
  // double radius;
};

#endif

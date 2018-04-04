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

#ifndef _waypoint_h_
#define _waypoint_h_

#include <pedsim/ped_waypoint.h>
#include <pedsim_simulator/element/scenarioelement.h>

class Waypoint : public ScenarioElement, public Ped::Twaypoint {
  Q_OBJECT

 public:
  explicit Waypoint(const QString& nameIn);
  Waypoint(const QString& nameIn, const Ped::Tvector& positionIn);

  virtual ~Waypoint();

  // Signals
 signals:
  void positionChanged(double x, double y);

  // Methods
 public:
  QString getName() const;
  // → Ped::Twaypoint Overrides
  virtual void setPosition(double xIn, double yIn);
  virtual void setPosition(const Ped::Tvector& posIn);
  virtual void setx(double xIn);
  virtual void sety(double yIn);
  virtual void setRadius(double rIn);

  // Attributes
 protected:
  const QString name;
};

#endif

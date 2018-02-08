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

#ifndef _attractionarea_h_
#define _attractionarea_h_

#include <pedsim/ped_vector.h>
#include <pedsim_simulator/element/scenarioelement.h>
#include <QSizeF>

class AttractionArea : public ScenarioElement {
  Q_OBJECT

  // Constructor and Destructor
 public:
  AttractionArea(const QString& nameIn);
  virtual ~AttractionArea();

  // Signals
 signals:
  void positionChanged(double x, double y);
  void sizeChanged(double width, double height);
  void strengthChanged(double strength);

  // Methods
 public:
  QString getName() const;
  // → Position
  virtual Ped::Tvector getPosition() const;
  virtual void setPosition(double xIn, double yIn);
  virtual void setPosition(const Ped::Tvector& posIn);
  virtual void setx(double xIn);
  virtual void sety(double yIn);
  // → Size
  virtual QSizeF getSize() const;
  virtual void setSize(double widthIn, double heightIn);
  virtual void setSize(const QSizeF& sizeIn);
  virtual void setWidth(double widthIn);
  virtual void setHeight(double heightIn);
  // → Attraction
  virtual double getStrength() const;
  virtual void setStrength(double strengthIn);

  int getId() { return id_; }

  // → ScenarioElement Overrides/Overloads
 public:
  // 	virtual QPointF getVisiblePosition() const;
  // 	virtual void setVisiblePosition(const QPointF& positionIn);
  virtual QString toString() const;

  // Attributes
 protected:
  const QString name;
  Ped::Tvector position;
  QSizeF size;
  double attractionStrength;
  int id_;
};

#endif

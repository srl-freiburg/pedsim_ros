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

#include <pedsim_simulator/element/attractionarea.h>

AttractionArea::AttractionArea(const QString& nameIn) : name(nameIn) {
  static int staticid = 1000;
  id_ = staticid++;
  // initialize values
  size.setWidth(0);
  size.setHeight(0);
  attractionStrength = 0;
}

AttractionArea::~AttractionArea() {}

QString AttractionArea::getName() const { return name; }

Ped::Tvector AttractionArea::getPosition() const { return position; }

void AttractionArea::setPosition(double xIn, double yIn) {
  position.x = xIn;
  position.y = yIn;

  // inform users
  emit positionChanged(position.x, position.y);
}

void AttractionArea::setPosition(const Ped::Tvector& posIn) {
  position = posIn;

  // inform users
  emit positionChanged(position.x, position.y);
}

void AttractionArea::setx(double xIn) {
  position.x = xIn;

  // inform users
  emit positionChanged(position.x, position.y);
}

void AttractionArea::sety(double yIn) {
  position.y = yIn;

  // inform users
  emit positionChanged(position.x, position.y);
}

QSizeF AttractionArea::getSize() const { return size; }

void AttractionArea::setSize(double widthIn, double heightIn) {
  size.setWidth(widthIn);
  size.setHeight(heightIn);

  // inform users
  emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setSize(const QSizeF& sizeIn) {
  size = sizeIn;

  // inform users
  emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setWidth(double widthIn) {
  size.setWidth(widthIn);

  // inform users
  emit sizeChanged(size.width(), size.height());
}

void AttractionArea::setHeight(double heightIn) {
  size.setHeight(heightIn);

  // inform users
  emit sizeChanged(size.width(), size.height());
}

double AttractionArea::getStrength() const { return attractionStrength; }

void AttractionArea::setStrength(double strengthIn) {
  attractionStrength = strengthIn;

  // inform users
  emit strengthChanged(attractionStrength);
}

QString AttractionArea::toString() const {
  return tr("AttractionArea: '%1' (@%2, %3)")
      .arg(name)
      .arg(position.x)
      .arg(position.y);
}

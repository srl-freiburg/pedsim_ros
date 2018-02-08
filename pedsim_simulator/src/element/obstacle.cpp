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

#include <pedsim_simulator/element/obstacle.h>

Obstacle::Obstacle(double pax, double pay, double pbx, double pby)
    : Tobstacle(pax, pay, pbx, pby){};

Obstacle::~Obstacle() {}

/// moves the obstacle to a new position
void Obstacle::setPosition(double pax, double pay, double pbx, double pby) {
  Tobstacle::setPosition(pax, pay, pbx, pby);

  // inform users
  emit positionChanged();
}

void Obstacle::setPosition(const QPointF& startIn, const QPointF& endIn) {
  setPosition(startIn.x(), startIn.y(), endIn.x(), endIn.y());
}

void Obstacle::setX1(double xIn) {
  // update x1, keep the other values
  setPosition(xIn, getay(), getbx(), getby());
}

void Obstacle::setY1(double yIn) {
  // update y1, keep the other values
  setPosition(getax(), yIn, getbx(), getby());
}

void Obstacle::setX2(double xIn) {
  // update y2, keep the other values
  setPosition(getax(), getay(), xIn, getby());
}

void Obstacle::setY2(double yIn) {
  // update x2, keep the other values
  setPosition(getax(), getay(), getbx(), yIn);
}

QPointF Obstacle::getVisiblePosition() const {
  return QPointF(getax(), getay());
}

void Obstacle::setVisiblePosition(const QPointF& positionIn) {
  // compute new end position
  QPointF deltaPos(positionIn.x() - getax(), positionIn.y() - getay());
  QPointF endPos = QPointF(getbx(), getby()) + deltaPos;

  // set new position
  setPosition(positionIn.x(), positionIn.y(), endPos.x(), endPos.y());

  // inform users
  emit positionChanged();
}

QString Obstacle::toString() const {
  return tr("Obstacle (%1,%2 - %3,%4)")
      .arg(getax())
      .arg(getay())
      .arg(getbx())
      .arg(getby());
}

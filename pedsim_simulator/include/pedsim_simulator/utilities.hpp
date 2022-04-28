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
 */

#ifndef UTILITIES_H
#define UTILITIES_H

/// --------------------------------------
/// \enum RobotMode
/// \brief Robot control mode
/// --------------------------------------
enum class RobotMode { CONTROLLED = 0, TELEOPERATION = 1, SOCIAL_DRIVE = 2 };

/// \brief Modes for running the simulator
enum class VisualMode { HEADLESS = 0, MINIMAL = 1, FULL = 2 };

/// --------------------------------------
/// \struct Location
/// \brief 2D location/cell
/// --------------------------------------
struct Location {
  float x;
  float y;

  Location(float xx, float yy) : x(xx), y(yy) {}

  bool operator==(Location a) {
    if (x == a.x && y == a.y)
      return true;
    else
      return false;
  }
};

#endif

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
* \author Luigi Palmieri <palmieri@cs.uni-freiburg.de>
*/

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

class OrientationHandler {
 private:
  /// Quaternion elements
  double qx_;
  double qy_;
  double qz_;
  double qw_;

  /// robot's heading angle
  double theta_;

  /// Rotation Matrix
  Eigen::Matrix3f rotation_matrix_;

 public:
  OrientationHandler() {
    qx_ = 0.0;
    qy_ = 0.0;
    qz_ = 0.0;
    qw_ = 1.0;
    theta_ = 0.0;
  }
  /// Constructor to call if you know the heading of the robot in a xy plane
  explicit OrientationHandler(double theta);
  /// Constructor to call if you know the velocity's components of the robot in
  /// a xy plane
  OrientationHandler(double vx, double vy);
  /// Constructor to call if you know the quaternion's components
  OrientationHandler(double qw, double qx, double qy, double qz);
  ~OrientationHandler();

  /// Computes and returns the robot's heading angle Theta (in a xy plane) given
  /// its quaternion
  double getTheta(Eigen::Quaternionf quaternion);
  Eigen::Quaternionf angle2Quaternion(double theta);
  Eigen::Quaternionf rpy2Quaternion(double roll, double pitch, double yaw);

  /// get the robot's heading angle Theta computed by the constructor
  double gett();

  /// get the quaternion x component of the current robot orientation
  double getQx();
  /// get the quaternion y component of the current robot orientation
  double getQy();
  /// get the quaternion z component of the current robot orientation
  double getQz();
  /// get the quaternion w component of the current robot orientation
  double getQw();
};

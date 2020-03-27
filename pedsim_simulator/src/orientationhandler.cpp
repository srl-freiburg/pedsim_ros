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

#include <pedsim_simulator/orientationhandler.hpp>

OrientationHandler::OrientationHandler(double theta) {
  // Below is setup stuff
  Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
  r_m(0, 0) = cos(theta);
  r_m(0, 1) = -sin(theta);
  r_m(0, 2) = 0;
  r_m(1, 0) = sin(theta);
  r_m(1, 1) = cos(theta);
  r_m(1, 2) = 0;
  r_m(2, 0) = 0;
  r_m(2, 1) = 0;
  r_m(2, 2) = 1;

  Eigen::Quaternionf quaternion(r_m);
  qw_ = quaternion.w();
  qx_ = quaternion.x();
  qy_ = quaternion.y();
  qz_ = quaternion.z();
  rotation_matrix_ = Eigen::Matrix3f(r_m);
}

OrientationHandler::OrientationHandler(double qw, double qx, double qy,
                                       double qz) {
  qw_ = qw;
  qx_ = qx;
  qy_ = qy;
  qz_ = qz;
  Eigen::Quaternionf quaternion(qw, qx, qy, qz);
  theta_ = getTheta(quaternion);
}

OrientationHandler::OrientationHandler(double vx, double vy) {
  const double theta = atan2(vy, vy);

  Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
  r_m(0, 0) = cos(theta);
  r_m(0, 1) = -sin(theta);
  r_m(0, 2) = 0;
  r_m(1, 0) = sin(theta);
  r_m(1, 1) = cos(theta);
  r_m(1, 2) = 0;
  r_m(2, 0) = 0;
  r_m(2, 1) = 0;
  r_m(2, 2) = 1;

  // The important part: Construct a quaternion from the matrix of eigenvectors
  // (which is a rotation matrix)
  Eigen::Quaternionf quaternion_notNorm(r_m);
  Eigen::Quaternionf quaternion = quaternion_notNorm.normalized();

  qw_ = quaternion.w();
  qx_ = quaternion.x();
  qy_ = quaternion.y();
  qz_ = quaternion.z();
  rotation_matrix_ = Eigen::Matrix3f(r_m);
}

OrientationHandler::~OrientationHandler() {}

double OrientationHandler::getTheta(Eigen::Quaternionf quaternion) {
  Eigen::Matrix3f rm = quaternion.toRotationMatrix();
  return acos(rm(0, 0));
}

Eigen::Quaternionf OrientationHandler::angle2Quaternion(double theta) {
  Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
  r_m(0, 0) = cos(theta);
  r_m(0, 1) = -sin(theta);
  r_m(0, 2) = 0;
  r_m(1, 0) = sin(theta);
  r_m(1, 1) = cos(theta);
  r_m(1, 2) = 0;
  r_m(2, 0) = 0;
  r_m(2, 1) = 0;
  r_m(2, 2) = 1;

  // The important part: Construct a quaternion from the matrix of eigenvectors
  // (which is a rotation matrix)
  Eigen::Quaternionf quaternion_notNorm(r_m);
  Eigen::Quaternionf quaternion = quaternion_notNorm.normalized();

  return quaternion;
}

Eigen::Quaternionf OrientationHandler::rpy2Quaternion(double roll, double pitch,
                                                      double yaw) {
  Eigen::Quaternionf r_m = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return r_m.normalized();
}

double OrientationHandler::getQx() { return qx_; }

double OrientationHandler::getQy() { return qy_; }

double OrientationHandler::getQz() { return qz_; }

double OrientationHandler::getQw() { return qw_; }

double OrientationHandler::gett() { return theta_; }

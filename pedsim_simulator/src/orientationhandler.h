/// Simple Orientation Handler for a Wheeled Mobile Robot
/// Once you construct the object, you can get the quaternion's elements
/// Normalized Quaternion Elements:
///  qx_,  qy_,  qz_,  qw_;
/// Rotation Matrix
/// rotation_matrix (Eigen::Matrix3f)
///	
/// Copyright (C) Social Robotics Lab, Uni Freiburg
/// \author Luigi Palmieri
/// \author Billy Okal 


#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>

#include <boost/shared_ptr.hpp>


class OrientationHandler
{

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

	OrientationHandler() { qx_ = 0.0; qy_ = 0.0; qz_ = 0.0; qw_ = 1.0; }

	/// Constructor to call if you know the heading of the robot in a xy plane 
	OrientationHandler(double theta);
	/// Constructor to call if you know the velocity's components of the robot in a xy plane
	OrientationHandler(double vx, double vy);
	/// Constructor to call if you know the quaternion's components 
	OrientationHandler(double qw, double qx, double qy, double qz);

	~OrientationHandler();
	/// Computes and returns the robot's heading angle Theta (in a xy plane) given its quaternion
	double getTheta(Eigen::Quaternionf quaternion);

	Eigen::Quaternionf angle2Quaternion(double theta);

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


typedef boost::shared_ptr<OrientationHandler> OrientationHandlerPtr;
typedef boost::shared_ptr<OrientationHandler const> OrientationHandlerConstPtr;

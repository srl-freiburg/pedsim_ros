#include <pedsim_simulator/orientationhandler.h>

OrientationHandler::OrientationHandler(double theta)
{
    //Below is setup stuff
    Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
    r_m(0,0) = cos(theta);
    r_m(0,1) = -sin(theta);
    r_m(0,2) = 0;
    r_m(1,0) = sin(theta);
    r_m(1,1) = cos(theta);
    r_m(1,2) = 0;
    r_m(2,0) = 0;
    r_m(2,1) = 0;
    r_m(2,2) = 1;
   
    Eigen::Quaternionf quaternion (r_m);
    qw_= quaternion.w();
    qx_=quaternion.x();
    qy_=quaternion.y();
    qz_=quaternion.z();    
    rotation_matrix_=Eigen::Matrix3f(r_m);
}

OrientationHandler::OrientationHandler(double qw, double qx, double qy, double qz)
{
    qw_=qw;
    qx_=qx;
    qy_=qy;
    qz_=qz;
    Eigen::Quaternionf quaternion(qw,qx,qy,qz);
    theta_=getTheta(quaternion);
}


OrientationHandler::OrientationHandler(double vx,double vy)
{
    double theta=0;

    theta = atan2(vy, vy);

    Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
    r_m(0,0) = cos(theta);
    r_m(0,1) = -sin(theta);
    r_m(0,2) = 0;
    r_m(1,0) = sin(theta);
    r_m(1,1) = cos(theta);
    r_m(1,2) = 0;
    r_m(2,0) = 0;
    r_m(2,1) = 0;
    r_m(2,2) = 1;
   
    //The important part: Construct a quaternion from the matrix of eigenvectors (which is a rotation matrix)
    Eigen::Quaternionf quaternion_notNorm (r_m);
    Eigen::Quaternionf quaternion= quaternion_notNorm.normalized();

    qw_=quaternion.w();
    qx_=quaternion.x();
    qy_=quaternion.y();
    qz_=quaternion.z();    
    rotation_matrix_=Eigen::Matrix3f(r_m);
}

OrientationHandler::~OrientationHandler()
{

}


double OrientationHandler::getTheta(Eigen::Quaternionf quaternion)
{
    Eigen::Matrix3f rm =quaternion.toRotationMatrix();
    return acos(rm(0,0));
}

Eigen::Quaternionf OrientationHandler::angle2Quaternion(double theta)
{
    Eigen::Matrix3f r_m(Eigen::Matrix3f::Identity());
    r_m(0,0) = cos(theta);
    r_m(0,1) = -sin(theta);
    r_m(0,2) = 0;
    r_m(1,0) = sin(theta);
    r_m(1,1) = cos(theta);
    r_m(1,2) = 0;
    r_m(2,0) = 0;
    r_m(2,1) = 0;
    r_m(2,2) = 1;

    //The important part: Construct a quaternion from the matrix of eigenvectors (which is a rotation matrix)
    Eigen::Quaternionf quaternion_notNorm (r_m);
    Eigen::Quaternionf quaternion = quaternion_notNorm.normalized();
    
    return quaternion;
}

double OrientationHandler::getQx()
{
        return qx_;
}
    


double OrientationHandler::getQy()
{
        return qy_;
}
    

double OrientationHandler::getQz()
{
        return qz_;
}
    


double OrientationHandler::getQw(){
        return qw_;
}

double OrientationHandler::gett()
{
    return theta_;
}
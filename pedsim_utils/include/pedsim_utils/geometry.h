

#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace pedsim {

geometry_msgs::Quaternion angleToQuaternion(const double theta);

geometry_msgs::Quaternion rpyToQuaternion(const double roll, const double pitch,
                                          const double yaw);

geometry_msgs::Quaternion toQuaternionMsg(const Eigen::Quaternionf& quaternion);

geometry_msgs::Quaternion poseFrom2DVelocity(const double vx, const double vy);

std::vector<std::pair<float, float>> LineObstacleToCells(const float x1,
                                                         const float y1,
                                                         const float x2,
                                                         const float y2);

}  // namespace pedsim



#include <geometry_msgs/msg/quaternion.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace geometry_msgs::msg;

namespace pedsim {

Quaternion angleToQuaternion(const double theta);

Quaternion rpyToQuaternion(const double roll, const double pitch,
                                          const double yaw);

Quaternion toQuaternionMsg(const Eigen::Quaternionf& quaternion);

Quaternion poseFrom2DVelocity(const double vx, const double vy);

std::vector<std::pair<float, float>> LineObstacleToCells(const float x1,
                                                         const float y1,
                                                         const float x2,
                                                         const float y2,
                                                         const float resolution);

}  // namespace pedsim

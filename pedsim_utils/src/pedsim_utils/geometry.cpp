
#include <pedsim_utils/geometry.h>

namespace pedsim {

geometry_msgs::Quaternion angleToQuaternion(const double theta) {
  Eigen::Matrix3f rotation_matrix(Eigen::Matrix3f::Identity());
  rotation_matrix(0, 0) = std::cos(theta);
  rotation_matrix(0, 1) = -std::sin(theta);
  rotation_matrix(0, 2) = 0;
  rotation_matrix(1, 0) = std::sin(theta);
  rotation_matrix(1, 1) = std::cos(theta);
  rotation_matrix(1, 2) = 0;
  rotation_matrix(2, 0) = 0;
  rotation_matrix(2, 1) = 0;
  rotation_matrix(2, 2) = 1;

  Eigen::Quaternionf quaternion(rotation_matrix);
  return toQuaternionMsg(quaternion.normalized());
}

geometry_msgs::Quaternion rpyToQuaternion(const double roll, const double pitch,
                                          const double yaw) {
  Eigen::Quaternionf r_m = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return toQuaternionMsg(r_m.normalized());
}

geometry_msgs::Quaternion toQuaternionMsg(
    const Eigen::Quaternionf& quaternion) {
  geometry_msgs::Quaternion gq;
  gq.x = quaternion.x();
  gq.y = quaternion.y();
  gq.z = quaternion.z();
  gq.w = quaternion.w();
  return std::move(gq);
}

geometry_msgs::Quaternion poseFrom2DVelocity(const double vx, const double vy) {
  const double theta = std::atan2(vy, vx);
  return angleToQuaternion(theta);
}

// Based on https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
std::vector<std::pair<float, float>> LineObstacleToCells(const float x1,
                                                         const float y1,
                                                         const float x2,
                                                         const float y2) {
  std::vector<std::pair<float, float>> obstacle_cells;
  float dx = x2 - x1;
  float dy = y2 - y1;
  float step = std::fabs(dx) >= std::fabs(dy) ? std::fabs(dx) : std::fabs(dy);
  float x = x1;
  float y = y1;
  if (step > 0) {
    dx /= step;
    dy /= step;
    int i = 1;
    while (i <= step) {
      obstacle_cells.emplace_back(x, y);
      x = x + dx;
      y = y + dy;
      i = i + 1;
    }
  }
  obstacle_cells.emplace_back(x, y);
  return obstacle_cells;
}

}  // namespace pedsim

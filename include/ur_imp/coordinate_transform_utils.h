#ifndef COORDINATE_TRANSFORM_UTILS_H
#define COORDINATE_TRANSFORM_UTILS_H

#include <eigen3/Eigen/Eigen>

namespace CoordTransformUtils{

  // make the sign of quaternion consistent
  bool checkQuaternionSign(const Eigen::Quaterniond &quat);
  Eigen::Quaterniond flipQuaternionSign(const Eigen::Quaterniond &quat);
  Eigen::Quaterniond checkFlipQuaternionSign(const Eigen::Quaterniond &quat);

  // rotation matrix around axis
  Eigen::Matrix3d rotX(double angle);
  Eigen::Matrix3d rotY(double angle);
  Eigen::Matrix3d rotZ(double angle);

  // RPY <-> rotation matrix
  Eigen::Matrix3d RPY2RotMat(const Eigen::Vector3d &RPY);
  Eigen::Vector3d rotMat2RPY(const Eigen::Matrix3d &R);

  // Euler angles -> rotation matrix
  Eigen::Matrix3d zyxEuler2RotMat(const Eigen::Vector3d &vAngle);

  // Orientation error
  Eigen::Vector3d quaternionError(const Eigen::Quaterniond &quat_des,
                                  const Eigen::Quaterniond &quat_current);

  Eigen::Vector3d quaternionLogError(const Eigen::Quaterniond &quat_des,
                                     const Eigen::Quaterniond &quat_current);

  // omega to skew symmetric matrix
  Eigen::Matrix3d omega2SkewSymmetricMat(const Eigen::Vector3d &w);

  // quaternion multiplication
  Eigen::Quaterniond quaternionMult(const Eigen::Quaterniond &quat_q,
                                    const Eigen::Quaterniond &quat_p);

}

#endif // COORDINATE_TRANSFORM_UTILS_H

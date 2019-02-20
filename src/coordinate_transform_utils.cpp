/**********************************************************************************
  coordinate_transform_utils.cpp

  math utilities for coordinate transform

  September 2015, Jun Nakanishi
**********************************************************************************/

#include <ur_imp/coordinate_transform_utils.h>
#include <ros/ros.h>
#include <ur_imp/my_utils.h>

/**********************************************************************************
  check sign of quaternion
**********************************************************************************/
bool CoordTransformUtils::checkQuaternionSign(const Eigen::Quaterniond &quat){
  // return true if w < 0

  if(quat.w()<0.0){
      return true;
    }

  return false;
}

/**********************************************************************************
  flip sign of quaternion
**********************************************************************************/
Eigen::Quaterniond CoordTransformUtils::flipQuaternionSign(const Eigen::Quaterniond &quat){

  return Eigen::Quaterniond(-quat.w(), -quat.x(), -quat.y(), -quat.z());

}

/**********************************************************************************
  check and flip sign of quaternion if necessary
**********************************************************************************/
Eigen::Quaterniond CoordTransformUtils::checkFlipQuaternionSign(const Eigen::Quaterniond &quat){

  if(checkQuaternionSign(quat)){
      return flipQuaternionSign(quat);
    }

  return quat;

}

/**********************************************************************************
  RPY to rotation matrix
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::RPY2RotMat(const Eigen::Vector3d &RPY){

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  return R = Eigen::AngleAxisd(RPY(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(RPY(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(RPY(0), Eigen::Vector3d::UnitX());
}

/**********************************************************************************
  rotation matrix to RPY
**********************************************************************************/
Eigen::Vector3d CoordTransformUtils::rotMat2RPY(const Eigen::Matrix3d &R){

  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler = R.eulerAngles(2,1,0); // zyx euler angles

  rpy(0) = euler(2); // roll  = eular z
  rpy(1) = euler(1); // pitch = eular y
  rpy(2) = euler(0); // yaw   = eular x

  return rpy;
}

/**********************************************************************************
  zyx Euler Angles to rotation matrix
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::zyxEuler2RotMat(const Eigen::Vector3d &vAngle){

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  return R = Eigen::AngleAxisd(vAngle(2), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(vAngle(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(vAngle(0), Eigen::Vector3d::UnitX());
}

/**********************************************************************************
  rotation matrix around x axis
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::rotX(double angle){

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  return R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
}


/**********************************************************************************
  rotation matrix around y axis
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::rotY(double angle){

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  return R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
}


/**********************************************************************************
  rotation matrix around z axis
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::rotZ(double angle){

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  return R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
}

/**********************************************************************************
  quaternion error (Siciliano (3.87))
**********************************************************************************/
Eigen::Vector3d CoordTransformUtils::quaternionError(const Eigen::Quaterniond &quat_des,
                                                     const Eigen::Quaterniond &quat_current){

  // Eigen::Matrix3d S = omega2SkewSymmetricMat(quat_des.vec());
  // Eigen::Vector3d eo = quat_current.w()*quat_des.vec() - quat_des.w()*quat_current.vec() - S*quat_current.vec();

  Eigen::Quaterniond delta_q = quat_des * quat_current.inverse();
  delta_q.normalize(); // normalize quaternion
  delta_q = CoordTransformUtils::checkFlipQuaternionSign(delta_q); // flip sign if necessary

  Eigen::Vector3d eo = delta_q.vec();

  return eo;
}

/**********************************************************************************
  quaternion log error (Shoemake, Murray, Righetti)
**********************************************************************************/
// definition 1 (cf. Shoemake)
// exp(q) = [cos(th), v*sin(th)]
// log(q) = [0, v*th]
// angular velocity: around v with 2*th
//
// th = acos(w)
// sin(th) = sin(acos(w))
// v = q.vec()/sin(th)
//
// eo = 2*v*th = 2*q.vec()/sin(acos(w)) * acos(w)
//             = 2*q.vec()/sinc(a), where a = acos(w)

// definition 2 (Murray, Righetti)
// q = [cos(th/2), v*sin(th/2)]
// log(q) = [0, v*th]
// angular velocity: around v with th
//
// th = 2*acos(w)
// sin(th/2) = sin(2*acos(w)/2) = sin(acos(w))
// v = q.vec()/sin(th/2)
//
// eo = v*th = q.vec()/sin(acos(w)) * 2*acos(w)
//           = 2*q.vec()/sin(acos(w)) * acos(w)
//           = 2*q.vec()/sinc(a), where a = acos(w)

// avoid division by zero when sin(acos(w)) ~= 0
// Taylor expansion of th/sin(th)   = 1 + 1/6*th^2
//                     th/sin(th/2) = 2 + 1/12*th^2

// caevat: to avoid division by 0 (singularity), it is necessary to consider two cases
// 1) sin(0) with quat.w() = 1. In this case, use Taylor expansion of th/sin(th) = 2 + 1/12*th^2 or simply 2
// 2) sin(pi) with quat.w() = -1. This shouldn't happen since we restrict quat.w() >= 0.


Eigen::Vector3d CoordTransformUtils::quaternionLogError(const Eigen::Quaterniond &quat_des,
                                                        const Eigen::Quaterniond &quat_current){

  const double threshold = 0.001;
  Eigen::Vector3d eo;

  Eigen::Quaterniond delta_q = quat_des * quat_current.inverse();
  delta_q = CoordTransformUtils::checkFlipQuaternionSign(delta_q); // flip sign if needed to make sure delta_q.w() > 0

  // acos(x) with x>1 returns nan. Thus, some care will be necessary to make sure x<=1
  // 1. normalize delta_q
  // 2. use atan2(delta_q.vec().norm(), delta_q.w()) instead for more nuemrical robustness

  delta_q.normalize(); // normalize quaternion
  //  double theta = 2.0 * acos(delta_q.w());
  double theta = 2.0 * atan2(delta_q.vec().norm(), delta_q.w());
  //  ROS_INFO_STREAM("theta = " << theta);

  // avoid division by 0
  if(abs(1.0-(delta_q.w()*delta_q.w()) < threshold)){
      // Taylor expansion of th/sin(th) ~= 1 + 1/6*th^2
      // and th/(sin(th/2)) ~= 2 + 1/12*th^2
    // eo = (2.0 + 1.0/12.0 * theta*theta) * delta_q.vec(); // up to the second order
    eo = 2.0 * delta_q.vec(); // up to the first order (first order term is zero)
    }
  else{
      eo = theta/sin(theta/2.0) * delta_q.vec();
    }

  return eo;
}

/**********************************************************************************
  skew-symmetric operator for given angular velocity omega
**********************************************************************************/
Eigen::Matrix3d CoordTransformUtils::omega2SkewSymmetricMat(const Eigen::Vector3d &w){

  Eigen::Matrix3d S;

  S <<   0, -w(2),  w(1),
      w(2),    0 , -w(0),
      -w(1),  w(0),    0;

  return S;
}

/**********************************************************************************
  quaternion multiplication (Murray book p. 33)
**********************************************************************************/
Eigen::Quaterniond CoordTransformUtils::quaternionMult(const Eigen::Quaterniond &quat_q,
                                                       const Eigen::Quaterniond &quat_p){

  Eigen::Quaterniond quat_qp;

  quat_qp.w()   = quat_q.w()*quat_p.w() - quat_q.vec().dot(quat_p.vec());
  quat_qp.vec() = quat_q.w()*quat_p.vec() + quat_p.w()*quat_q.vec() + quat_q.vec().cross(quat_p.vec());

  return quat_qp;
}

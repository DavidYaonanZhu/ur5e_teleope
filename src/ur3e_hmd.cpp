#include "ur3e_hmd/ur3e_hmd.h"
#include <cmath>

namespace ur_nu {

UR3e_hmd::UR3e_hmd(ros::NodeHandle &node_handle) {
  nh_ = node_handle;

  sub_hmd_pose_ =
      nh_.subscribe("/oculus_rift", 1, &UR3e_hmd::getHMDPoseCb, this);

  pub_robot_tar_pose_cmd_ =
      nh_.advertise<geometry_msgs::PoseStamped>("ur3e_hmd/robot_tar_pose", 1);
  hmd_pose_ = VectorXd::Zero(7);
  flag_omega_rcv_ = false;
  firsttime_ = true;
  // ur3e initial pose
  init_x_ = 0.054819;  //
  init_y_ = 0.392007;
  init_z_ = 0.364382;

  // initialize hmd orientation in euler angle
  hmd_roll_ = 0, hmd_pitch_ = 0, hmd_yaw_ = 0;
  // initialize orientation regulation
  reg_roll_ = 1.047, reg_pitch_ = 1.047, reg_yaw_ = 1.047;

  // initialize quaternion
  // quat_rot_hmd2robot_ = Eigen::Quaterniond::Identity();
  quat_hmd_ = Eigen::Quaterniond::Identity();
  quat_hmd_tmp_ = Eigen::Quaterniond::Identity();
  // rotation matrix hmd2robot
  //  rot_matrix_hmd2robot_=Eigen::Matrix3d::Zero();
  rot_matrix_hmd2world_(0, 0) = 1;
  rot_matrix_hmd2world_(1, 2) = 1;
  rot_matrix_hmd2world_(2, 1) = -1;

  rot_matrix_hmd2robot_(0, 2) =
      -1;  // 0 -1 0, 0 0 1, -1 0 0 without switching quaternion yz
  rot_matrix_hmd2robot_(1, 0) = 1;
  rot_matrix_hmd2robot_(2, 1) = -1;

  quat_rot_hmd2robot_ = rot_matrix_hmd2robot_;
  quat_rot_hmd2world_ = rot_matrix_hmd2world_;
}

UR3e_hmd::~UR3e_hmd() {}

// Callbacks
void UR3e_hmd::getHMDPoseCb(const oculus_msgs::Oculus_rift::ConstPtr &msg) {
  pub_robot_tar_pose_msg_.pose = msg->pose;
  flag_omega_rcv_ = true;
}

void UR3e_hmd::control_loop() {
  if (flag_omega_rcv_) {
    std_msgs::Header header_tmp;
    header_tmp.stamp = ros::Time::now();

    pub_robot_tar_pose_msg_.header = header_tmp;
    robot_tar_pose_msg_tmp_ = pub_robot_tar_pose_msg_;
    // ROS_INFO_STREAM("firsttime_:" << firsttime_);
    if (firsttime_) {
      init_hmd_x_ = robot_tar_pose_msg_tmp_.pose.position.x;
      init_hmd_y_ = robot_tar_pose_msg_tmp_.pose.position.y;
      init_hmd_z_ = robot_tar_pose_msg_tmp_.pose.position.z;
      firsttime_ = false;
    }
    // ROS_INFO_STREAM("firsttime_:" << firsttime_);
    // translation regulation
    // X
    if ((robot_tar_pose_msg_tmp_.pose.position.x - init_hmd_x_) > 0.1) {
      pub_robot_tar_pose_msg_.pose.position.x = init_x_ + (0.1);
    } else if ((robot_tar_pose_msg_tmp_.pose.position.x - init_hmd_x_) < -0.1) {
      pub_robot_tar_pose_msg_.pose.position.x = init_x_ + (-0.1);
    } else {
      pub_robot_tar_pose_msg_.pose.position.x =
          init_x_ + (robot_tar_pose_msg_tmp_.pose.position.x - init_hmd_x_);
    }

    // Y
    if ((robot_tar_pose_msg_tmp_.pose.position.z - init_hmd_z_) > 0.1) {
      pub_robot_tar_pose_msg_.pose.position.y = init_y_ + -(0.1);
    } else if ((robot_tar_pose_msg_tmp_.pose.position.z - init_hmd_z_) < -0.1) {
      pub_robot_tar_pose_msg_.pose.position.y = init_y_ + -(-0.1);
    } else {
      pub_robot_tar_pose_msg_.pose.position.y =
          init_y_ + -(robot_tar_pose_msg_tmp_.pose.position.z - init_hmd_z_);
    }

    // Z
    if ((robot_tar_pose_msg_tmp_.pose.position.y - init_hmd_y_) > 0.1) {
      pub_robot_tar_pose_msg_.pose.position.z = init_z_ + (0.1);
    } else if ((robot_tar_pose_msg_tmp_.pose.position.y - init_hmd_y_) < -0.1) {
      pub_robot_tar_pose_msg_.pose.position.z = init_z_ + (-0.1);
    } else {
      pub_robot_tar_pose_msg_.pose.position.z =
          init_z_ + (robot_tar_pose_msg_tmp_.pose.position.y - init_hmd_y_);
    }

    // mapping Translation
    //    pub_robot_tar_pose_msg_.pose.position.x =
    //        init_x_ +
    //           (robot_tar_pose_msg_tmp_.pose.position.x - init_hmd_x_);
    //    pub_robot_tar_pose_msg_.pose.position.y =
    //        init_y_ +
    //            -(robot_tar_pose_msg_tmp_.pose.position.z - init_hmd_z_);
    //    pub_robot_tar_pose_msg_.pose.position.z =
    //        init_z_ +
    //            (robot_tar_pose_msg_tmp_.pose.position.y - init_hmd_y_);

    quat_hmd_tmp_.x() = robot_tar_pose_msg_tmp_.pose.orientation.x;
    quat_hmd_tmp_.y() = robot_tar_pose_msg_tmp_.pose.orientation.y;
    quat_hmd_tmp_.z() = robot_tar_pose_msg_tmp_.pose.orientation.z;
    quat_hmd_tmp_.w() = robot_tar_pose_msg_tmp_.pose.orientation.w;

    hmd_euler_ = quat_hmd_tmp_.toRotationMatrix().eulerAngles(0, 1, 2);
    ROS_INFO_STREAM("hmd euler roll pitch yaw: "
                    << hmd_euler_[0]);  //<<hmd_euler_[1]<<hmd_euler_[2]);

    //        //regulation on yaw
    //        if (hmd_euler_[2] > reg_yaw_ ){
    //            hmd_euler_[2] = reg_yaw_;
    //        }
    //        else if (hmd_euler_[2] < -reg_yaw_){
    //            hmd_euler_[2] = -reg_yaw_;
    //        }

    //        if (hmd_euler_[1] > reg_pitch_ ){
    //            hmd_euler_[1] = reg_pitch_;
    //        }
    //        else if (hmd_euler_[1] < -reg_pitch_){
    //            hmd_euler_[1] = -reg_pitch_;
    //        }

    //        if (hmd_euler_[0] > reg_roll_ & hmd_euler_[0] < 3.14){
    //            hmd_euler_[0] = reg_roll_;
    //        }
    //        else if (hmd_euler_[0] > reg_roll_ & hmd_euler_[0] <3.14 ){
    //            hmd_euler_[0] = -reg_roll_;
    //        }

    // converting back regulated orientation to quaternion
    //        quat_hmd_tmp_ = Eigen::AngleAxisd(hmd_euler_[0],
    //        Eigen::Vector3d::UnitX())
    //                * Eigen::AngleAxisd(hmd_euler_[1],
    //                Eigen::Vector3d::UnitY())
    //                * Eigen::AngleAxisd(hmd_euler_[2],
    //                Eigen::Vector3d::UnitZ());

    //    //mapping rotation
    //    quat_hmd_.x()=robot_tar_pose_msg_tmp_.pose.orientation.x;
    //    quat_hmd_.y()=-robot_tar_pose_msg_tmp_.pose.orientation.z;
    //    quat_hmd_.z()=robot_tar_pose_msg_tmp_.pose.orientation.y;
    //    quat_hmd_.w()=robot_tar_pose_msg_tmp_.pose.orientation.w;

    quat_hmd_.x() = quat_hmd_tmp_.x();
    quat_hmd_.y() = -quat_hmd_tmp_.z();
    quat_hmd_.z() = quat_hmd_tmp_.y();
    quat_hmd_.w() = quat_hmd_tmp_.w();

    // rotation regulation
    //    hmd_euler_ = quat_hmd_.toRotationMatrix().eulerAngles(0, 1, 2);
    //    ROS_INFO_STREAM("hmd euler roll pitch yaw: "<<
    //    hmd_euler_[0]);//<<hmd_euler_[1]<<hmd_euler_[2]);

    //    //regulation on yaw
    //    if (hmd_euler_[2] > reg_yaw_ ){
    //        hmd_euler_[2] = reg_yaw_;
    //    }
    //    else if (hmd_euler_[2] < -reg_yaw_){
    //        hmd_euler_[2] = -reg_yaw_;
    //    }

    //    if (hmd_euler_[1] > reg_pitch_ ){
    //        hmd_euler_[1] = reg_pitch_;
    //    }
    //    else if (hmd_euler_[1] < -reg_pitch_){
    //        hmd_euler_[1] = -reg_pitch_;
    //    }

    //    if (hmd_euler_[0] > reg_roll_ ){
    //        hmd_euler_[0] = reg_roll_;
    //    }
    //    else if (hmd_euler_[0] < -reg_roll_){
    //        hmd_euler_[0] = -reg_roll_;
    //    }

    //    //converting back regulated orientation to quaternion
    //    quat_hmd_ = Eigen::AngleAxisd(hmd_euler_[0], Eigen::Vector3d::UnitX())
    //            * Eigen::AngleAxisd(hmd_euler_[1], Eigen::Vector3d::UnitY())
    //            * Eigen::AngleAxisd(hmd_euler_[2], Eigen::Vector3d::UnitZ());

    // quat_hmd_ = quat_hmd_ * quat_rot_hmd2world_;
    quat_hmd_ = quat_hmd_ * quat_rot_hmd2robot_;

    pub_robot_tar_pose_msg_.pose.orientation.x = quat_hmd_.x();
    pub_robot_tar_pose_msg_.pose.orientation.y = quat_hmd_.y();
    pub_robot_tar_pose_msg_.pose.orientation.z = quat_hmd_.z();
    pub_robot_tar_pose_msg_.pose.orientation.w = quat_hmd_.w();

    // ROS_INFO_STREAM("derivetiveX_:" <<
    // pub_robot_tar_pose_msg_.pose.position.x);

    // TF
    ur3e_TF_.header.stamp = header_tmp.stamp;
    ur3e_TF_.header.frame_id = "base_link";
    ur3e_TF_.child_frame_id = "oculus_hmd";
    ur3e_TF_.transform.translation.x = pub_robot_tar_pose_msg_.pose.position.x;
    ur3e_TF_.transform.translation.y = pub_robot_tar_pose_msg_.pose.position.y;
    ur3e_TF_.transform.translation.z = pub_robot_tar_pose_msg_.pose.position.z;
    // ur3e_TF_.transform.rotation.x =
    // pub_robot_tar_pose_msg_.pose.orientation.x;
    ur3e_TF_.transform.rotation.x = pub_robot_tar_pose_msg_.pose.orientation.x;
    ur3e_TF_.transform.rotation.y = pub_robot_tar_pose_msg_.pose.orientation.y;
    ur3e_TF_.transform.rotation.z = pub_robot_tar_pose_msg_.pose.orientation.z;
    ur3e_TF_.transform.rotation.w = pub_robot_tar_pose_msg_.pose.orientation.w;
    // tf情報をbroadcast(座標系の設定)
    hmd_TF_broadcaster_.sendTransform(ur3e_TF_);

    // robot target pos based on HMD pos
    pub_robot_tar_pose_cmd_.publish(pub_robot_tar_pose_msg_);
  }
}

}  // namespace ur_nu

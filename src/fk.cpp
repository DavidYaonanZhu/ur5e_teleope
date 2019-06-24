
#include "fk/fk.h"
#include <cmath>

//#include <signal.h>

using namespace ur_fk;


 namespace ur_fk {

URFK::URFK(ros::NodeHandle &node_handle) {
  nh_ = node_handle;

  sub_robot_pose_ =
      nh_.subscribe("joint_states", 1, &URFK::getRobotJPosCb, this);
//  sub_omega_pose_ =
//      nh_.subscribe("sigma7/sigma0/pose", 1, &URImp::getOmegaPoseCb, this);
//  sub_force_ = nh_.subscribe("sigma7/sigma0/force_filtered", 1,
//                             &URImp::getForceCb, this);
//  sub_force_robot_ = nh_.subscribe("wrench", 1, &URImp::getForceRobotCb, this);

//  pub_robot_fk_ =
//      nh_.advertise<geometry_msgs::PoseStamped>("ur_imp/robot_tar_pose", 1);
  pub_robot_current_pose_ =
      nh_.advertise<geometry_msgs::PoseStamped>("ur_fk/fk", 1);
//  pub_dx_pos_ = nh_.advertise<std_msgs::Float64MultiArray>("ur_imp/dx", 1);

//  pub_dx_pos_msg_.data.resize(3, 0);

//  // Virtual Stiffness
//  K_trans_ = Matrix3d::Identity();
//  K_trans_(0, 0) = 800;   // 0
//  K_trans_(1, 1) = 800;   // 0
//  K_trans_(2, 2) = 1500;  // 1500

//  K_rot_ = Matrix3d::Identity();
//  K_rot_(0, 0) = 10;  // 10
//  K_rot_(1, 1) = 10;  // 10
//  K_rot_(2, 2) = 10;  // 10

//  // Virtual Damping
//  C_trans_ = Matrix3d::Identity();
//  C_trans_(0, 0) =
//      20;  // Wacoh: 80 ATI17: 35 ATI40: 50 ATI40RobForceps: 150/400
//  C_trans_(1, 1) = 20;  // Wacoh: 80 ATI17: 35 ATI40: 50 ATI40RobForceps: 150/80
//  C_trans_(2, 2) =
//      20;  // Wacoh: 80 ATI17: 60 ATI40: 50 ATI40RobForceps:  150/400

//  C_rot_ = Matrix3d::Identity();
//  C_rot_(0, 0) = 1;  // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps: 0.7/0.5
//  C_rot_(1, 1) = 1;  // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps: 1.0/0.8
//  C_rot_(2, 2) = 1;  // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps: 1.0/0.5

//  // Virtual Mass
//  M_trans_ = Matrix3d::Identity();
//  M_trans_(0, 0) = 0.1;  // Wacoh 2 ATI17: 6 ATI40: 4.5 ATI40RobForceps: 4.5/8
//  M_trans_(1, 1) = 0.1;  // Wacoh 2 ATI17: 6 ATI40: 4.5 ATI40RobForceps: 4.5/8
//  M_trans_(2, 2) = 0.1;  // Wacoh 2 ATI17: 8 ATI40: 4.5 ATI40RobForceps: 4.5/8

//  M_rot_ = Matrix3d::Identity();
//  M_rot_(0, 0) =
//      0.02;  // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02
//  M_rot_(1, 1) =
//      0.02;  // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02
//  M_rot_(2, 2) =
//      0.02;  // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02

//  firsttime_ = true;
//  x = Eigen::VectorXd::Zero(6);
//  xd = Eigen::VectorXd::Zero(6);

//  //  q = Eigen::VectorXd::Zero(6);
//  //  qd = Eigen::VectorXd::Zero(6);

//  imp_m(0) = 0.1;
//  imp_m(1) = 0.1;
//  imp_m(2) = 0.1;

//  imp_d(0) = 200;
//  imp_d(1) = 200;
//  imp_d(2) = 200;

//  imp_k(0) = 300;
//  imp_k(1) = 300;
//  imp_k(2) = 300;

//  M = imp_m.asDiagonal();
//  D = imp_d.asDiagonal();
//  K = imp_k.asDiagonal();

//  A = Eigen::MatrixXd::Zero(6, 6);
//  B = Eigen::MatrixXd::Zero(6, 3);

//  A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
//  A.block(3, 0, 3, 3) = -M.inverse() * K;
//  A.block(3, 3, 3, 3) = -M.inverse() * D;
//  B.block(3, 0, 3, 3) = M.inverse();
//  imp_delta_pos_ = Eigen::Vector3d::Zero();

  // Local variables
  robot_j_pos_ = VectorXd::Zero(6);
  //omega_pose_ = VectorXd::Zero(7);
//  force_ = Vector3d::Zero();
//  torque_ = Vector3d::Zero();
//  robot_tar_cart_pos_ = Vector3d::Zero();
//  quat_d_ = Eigen::Quaterniond::Identity();
//  w_d_ = Eigen::Vector3d::Zero();
  flag_omega_rcv_ = false;
  force_robot_ok_ = false;

  urdf_param_ = "robot_description";
  double eps = 1e-5;
  tracik_solver_.reset(new TRAC_IK::TRAC_IK("base_link", "ee_link", urdf_param_,
                                            0.001, eps, TRAC_IK::Distance));

  /** KDL **/
  bool valid = tracik_solver_->getKDLChain(kdl_chain_);
  if (!valid) {
    ROS_ERROR_STREAM("There was no valid KDL chain found");
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
}
 }
int URFK::solveFK() {
  qtmp_.data = robot_j_pos_;

  bool kinematics_status;
  kinematics_status = fk_solver_->JntToCart(qtmp_, xtmp_);

  Affine3d T01;

  if (!kinematics_status) {
    tf::transformKDLToEigen(xtmp_, T01);
    R_base_eef = T01.rotation();
    T_base_eef = T01.translation();
    // ROS_INFO_STREAM("R01" << R_0_1_);
    // ROS_INFO_STREAM("T01" << T_0_1_);
    return 0;
  } else
    ROS_ERROR_STREAM("No FK");
  return -1;
}

// Callbacks
//void URImp::getForceCb(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
//  geometry_msgs::Wrench wrench_tmp;
//  wrench_tmp = msg->wrench;
//  force_[0] = wrench_tmp.force.x;
//  force_[1] = wrench_tmp.force.y;
//  force_[2] = wrench_tmp.force.z;
//  torque_[0] = wrench_tmp.torque.x;
//  torque_[1] = wrench_tmp.torque.y;
//  torque_[2] = wrench_tmp.torque.z;
//}

//void URImp::getForceRobotCb(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
//  geometry_msgs::Wrench wrench_tmp;
//  wrench_tmp = msg->wrench;
//  force_robot_[0] = wrench_tmp.force.x;
//  force_robot_[1] = wrench_tmp.force.y;
//  force_robot_[2] = wrench_tmp.force.z;
//  //  torque_[0] = wrench_tmp.torque.x;
//  //  torque_[1] = wrench_tmp.torque.y;
//  //  torque_[2] = wrench_tmp.torque.z;
//}

URFK::~URFK() {}

void URFK::getRobotJPosCb(const sensor_msgs::JointState::ConstPtr &msg) {
  robot_j_pos_ = VectorXd::Map(&msg->position[0], msg->position.size());

  Affine3d pose_eigen = Affine3d::Identity();
  geometry_msgs::Pose pose;
  solveFK();

  // Publish EE Pose

  pose_eigen.translation() = T_base_eef;
  pose_eigen.linear() = R_base_eef;
  tf::poseEigenToMsg(pose_eigen, pose);

  pub_robot_current_pose_msg_.header.stamp = ros::Time::now();
  pub_robot_current_pose_msg_.pose = pose;
  pub_robot_current_pose_.publish(pub_robot_current_pose_msg_);
}

//void URImp::getOmegaPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//  geometry_msgs::Pose pose_tmp;
//  pose_tmp = msg->pose;
//  geometry_msgs::Point position_tmp;
//  position_tmp = pose_tmp.position;
//  geometry_msgs::Quaternion orientation_tmp;
//  orientation_tmp = pose_tmp.orientation;

//  omega_pose_[0] = position_tmp.x;
//  omega_pose_[1] = position_tmp.y;
//  omega_pose_[2] = position_tmp.z;
//  omega_pose_[3] = orientation_tmp.x;
//  omega_pose_[4] = orientation_tmp.y;
//  omega_pose_[5] = orientation_tmp.z;
//  omega_pose_[6] = orientation_tmp.w;

//  flag_omega_rcv_ = true;
//}
//void URFK::control_loop(){

//}
//void URFK::control_loop() {
//  if (flag_omega_rcv_) {
//    // TRANSLATION
//    int i;
//    //            for (i = 2; i < 3; i++) {
//    //              V_d_[i] = (M_trans_(i, i) * V_temp_(i) +
//    //              thread_sampling_time_sec_d_ * force_[i]) /
//    //                          (thread_sampling_time_sec_d_ * C_trans_(i, i) +
//    //                          M_trans_(i, i)); // (m/s)
//    //            }
//    //            //    V_d_[2]=0.0; //ATI-> Problem with force Z when torque is
//    //            applied if (V_d_.norm() >= VEL_LIMIT_WAND)
//    //              V_d_ = V_d_ * (VEL_LIMIT_WAND / V_d_.norm()); // Translation
//    //              velocity software limit
//    //            ROS_INFO_STREAM("Vd (mm/s): " << 1000 * V_d_.transpose());

//    //            V_temp_ = V_d_; /* v[k-1] = v[k] */
//    //            imp_delta_pos_ = imp_delta_pos_ + thread_sampling_time_sec_d_
//    //            * V_d_; // (m) ROS_INFO_STREAM("delta_x (mm/s):" <<
//    //            1000*imp_delta_pos_.transpose());

//    // impedance model with stiffness K
//    for (i = 2; i < 3; i++) {
//      X_d_[i] = ((-2 * M_trans_(i, i) -
//                  C_trans_(i, i) * thread_sampling_time_sec_d_) *
//                     -1 * X_temp_1_(i) -
//                 X_temp_2_(i) * M_trans_(i, i) +
//                 thread_sampling_time_sec_d_ * thread_sampling_time_sec_d_ *
//                     force_[i]) /
//                (thread_sampling_time_sec_d_ * C_trans_(i, i) + M_trans_(i, i) +
//                 thread_sampling_time_sec_d_ * thread_sampling_time_sec_d_ *
//                     K_trans_(i, i));  // (m)
//    }
//    X_temp_2_ = X_temp_1_;
//    X_temp_1_ = X_d_;

//    //    Td_0_3_ = Td_0_3_ + 1000*thread_sampling_time_sec_d_*V_d_1_; // (mm)
//    imp_delta_pos_ = X_d_;  // (m)
//    ROS_INFO_STREAM("delta_x (m):" << imp_delta_pos_.transpose());

//    // ROTATION
//    Eigen::Quaterniond quat0 = Eigen::Quaterniond::Identity();
//    Eigen::VectorXd e_o = CoordTransformUtils::quaternionLogError(
//        quat_d_, quat0);  // Quaternion log error

//    Eigen::Vector3d torque_tmp = torque_;  // this part is for torque cordinates
//    //    torque_[1] = torque_tmp[0]; // with modified coedinates in omega
//    //    device. torque_[0] = -torque_tmp[1];
//    torque_[0] = -torque_tmp[2];  // pose90 torque remapping
//    torque_[2] = torque_tmp[0];   // pose90 torque remapping

//    w_dot_d_ = M_rot_.inverse() * (-C_rot_ * w_d_ - K_rot_ * e_o + torque_);
//    ROS_INFO_STREAM("Wa : (rad2/s) " << w_dot_d_.transpose());

//    w_d_ = w_d_ + w_dot_d_ * thread_sampling_time_sec_d_;

//    if (w_d_.norm() >= ANG_VEL_LIMIT_WAND)
//      w_d_ = w_d_ * (ANG_VEL_LIMIT_WAND /
//                     w_d_.norm());  // Translation velocity software limit
//    ROS_INFO_STREAM("Wd : (rad/s) " << w_d_.transpose());

//    Eigen::MatrixXd E_0_3;

//    Matrix3d Se(3, 3);  // Variable DESIRED quaternion vector skew matrix S(w)
//    // Calculation of S(e) matrix
//    Se << 0, -(quat_d_.vec())(2), (quat_d_.vec())(1), (quat_d_.vec())(2), 0,
//        -(quat_d_.vec())(0), -(quat_d_.vec())(1), (quat_d_.vec())(0), 0;

//    E_0_3 = MatrixXd::Identity(3, 3);
//    E_0_3 = quat_d_.w() * Matrix3d::Identity() - Se;

//    quat_d_.w() =
//        quat_d_.w() -
//        thread_sampling_time_sec_d_ * 0.5 *
//            ((quat_d_.vec())
//                 .dot(w_d_));  // DESIRED VRCM position in "Base Frame (F0)"
//    quat_d_.vec() =
//        quat_d_.vec() + thread_sampling_time_sec_d_ * 0.5 * E_0_3 *
//                            w_d_;  // DESIRED VRCM position in "Base Frame (F0)"
//    quat_d_.normalize();
//    ROS_INFO_STREAM("Quat des w:" << quat_d_.w()
//                                  << "vector: " << quat_d_.vec());

//    // impedance model: M xdd + D xd + K x = F
//    //                  I omegad + D_o omega + K_o e_o = torque
//    // state space representation

//    // static Eigen::Vector3d omega;
//    // static Eigen::Quaterniond quat;

//    //  ROS_INFO_STREAM("M: " << imp_m.transpose());
//    //  ROS_INFO_STREAM("D: " << imp_d.transpose());
//    //  ROS_INFO_STREAM("K: " << imp_k.transpose());
//    //  ROS_INFO_STREAM("I: " << imp_i.transpose());
//    //  ROS_INFO_STREAM("Do: " << imp_d_o.transpose());
//    //  ROS_INFO_STREAM("Ko: " << imp_k_o.transpose());

//    // in diagonal matrix form
//    // Eigen::Matrix3d I, Do, Ko;
//    //            M = M_trans_;
//    //            D = C_trans_;
//    //            K = K_trans_;
//    // ROS_INFO_STREAM("K:" << K.transpose());
//    //            M = imp_m.asDiagonal();
//    //            D = imp_d.asDiagonal();
//    //            K = imp_k.asDiagonal();
//    //            ROS_INFO_STREAM("K:" << K.transpose());

//    //            I = imp_i.asDiagonal();
//    //            Do = imp_d_o.asDiagonal();
//    //            Ko = imp_k_o.asDiagonal();

//    //            if(firsttime_){
//    //              x = Eigen::VectorXd::Zero(6);
//    //              //quat = Eigen::Quaterniond::Identity();
//    //              //omega = Eigen::Vector3d::Zero();

//    //              firsttime_ = false;
//    //            }

//    // translational impedance
//    // construct xd = A*x + B*F
//    // Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);
//    //  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

//    //            A.block(0,3,3,3) = Eigen::MatrixXd::Identity(3,3);
//    //            A.block(3,0,3,3) = -M.inverse() * K;
//    //            A.block(3,3,3,3) = -M.inverse() * D;
//    //            B.block(3,0,3,3) =  M.inverse();
//    //            ROS_INFO_STREAM("M inverse " << M.inverse());

//    //            //  ROS_INFO_STREAM("A=\n"<<A << "\nB = " << B);
//    //            // state space model of impedance dynamics
//    //            xd = A*x + B*force_;
//    //            ROS_INFO_STREAM("xd (m/s):" << xd.transpose());
//    //            ROS_INFO_STREAM("x (m/s):" << x.transpose());

//    //            //xd = A*x + B*F;

//    //            // integrate dynamics
//    //            //x = x + xd*this->dt;
//    //            x = x + xd*thread_sampling_time_sec_d_;
//    // xtemp =x  change x to xtemp

//    //            // return delta_target_pos
//    //            //delta_target_pos = x.head(3);
//    //            imp_delta_pos_ =x.head(3);
//    //            //imp_delta_pos_= delta_target_pos;
//    //            ROS_INFO_STREAM("delta_x (m):" << imp_delta_pos_.transpose());

//    robot_tar_cart_pos_[0] = omega_pose_[0] + imp_delta_pos_[0];
//    robot_tar_cart_pos_[1] = omega_pose_[1] + imp_delta_pos_[1];

//    if (force_robot_[2] < 15) {
//      force_robot_ok_ = true;

//    } else if (force_robot_[2] >= 20) {
//      force_robot_ok_ = false;
//    }

//    if (force_robot_ok_) {
//      robot_tar_cart_pos_[2] = omega_pose_[2] + imp_delta_pos_[2];

//    } else {
//      if (omega_pose_[2] + imp_delta_pos_[2] - robot_tar_cart_pos_[2] >= 0)
//        robot_tar_cart_pos_[2] = omega_pose_[2] + imp_delta_pos_[2];
//    }

//    Eigen::Quaterniond robot_ori_cart_pos;
//    Eigen::Quaterniond omega_tmp;

//    geometry_msgs::Point position_tmp;
//    geometry_msgs::Quaternion orientation_tmp;
//    std_msgs::Header header_tmp;
//    Eigen::Vector3d omega_vec;

//    omega_vec[0] = omega_pose_[3];
//    omega_vec[1] = omega_pose_[4];
//    omega_vec[2] = omega_pose_[5];

//    omega_tmp.w() = omega_pose_[6];
//    omega_tmp.vec() = omega_vec;

//    robot_ori_cart_pos = omega_tmp * quat_d_;

//    position_tmp.x = robot_tar_cart_pos_[0];
//    position_tmp.y = robot_tar_cart_pos_[1];
//    position_tmp.z = robot_tar_cart_pos_[2];

//    orientation_tmp.x = robot_ori_cart_pos.vec()[0];
//    orientation_tmp.y = robot_ori_cart_pos.vec()[1];
//    orientation_tmp.z = robot_ori_cart_pos.vec()[2];
//    orientation_tmp.w = robot_ori_cart_pos.w();

//    geometry_msgs::Pose pose_tmp;
//    pose_tmp.position = position_tmp;
//    pose_tmp.orientation = orientation_tmp;
//    header_tmp.stamp = ros::Time::now();

//    pub_robot_tar_pose_msg_.header = header_tmp;
//    pub_robot_tar_pose_msg_.pose = pose_tmp;

//    pub_dx_pos_msg_.data[0] = imp_delta_pos_[0];
//    pub_dx_pos_msg_.data[1] = imp_delta_pos_[1];
//    pub_dx_pos_msg_.data[2] = imp_delta_pos_[2];
//    pub_dx_pos_.publish(pub_dx_pos_msg_);

//    pub_robot_tar_pose_.publish(pub_robot_tar_pose_msg_);
//  }
//}

//}  // namespace ur_nu
// bool kill_process = false;
// void SigIntHandler(int signal)
//{
//  kill_process = true;
//  ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
//}

int main(int argc, char **argv) {
  int thread_sampling_freq;

  ros::init(argc, argv, "ur_fk");
  ros::NodeHandle node_handle;
  //signal(SIGINT, SigIntHandler);

  // Thread Sampling Frequency
  int thread_sampling_freq_hz;
  if (node_handle.getParam("interface_sampling_freq_hz",
                           thread_sampling_freq_hz)) {
    thread_sampling_freq = thread_sampling_freq_hz;
  } else
    thread_sampling_freq = 500;

  ros::Rate loop_rate(thread_sampling_freq);
  ROS_INFO_STREAM("thread sampling frequency: " << thread_sampling_freq
                                                << "hz");

  URFK fk(node_handle);

  while (ros::ok()) {
    //    if (in.getStatus() == I_RUNNING) {
    //      if (in.getPoseFK() == EXIT_SUCCESS)
    //        in.sendPoseCmd();
    //    }

    //fk.control_loop();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

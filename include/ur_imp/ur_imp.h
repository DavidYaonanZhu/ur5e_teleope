
#ifndef _UR_IMP_H_
#define _UR_IMP_H_

// ROS related
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>
//#include <unsupported/Eigen/EulerAngles>
// Services
//#include <impact_interface/InterfaceCmd.h>

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>


// Filters
//#include "filters.h"

using namespace Eigen;
using namespace std;

namespace ur_nu
{
const double VEL_LIMIT_WAND = 0.05;
const double DEG2RAD = 0.017453293;
const double PI = 3.14159265359;
const double thread_sampling_time_sec_d_ = 0.008;

class URImp
{

public:
  // Constructor
  URImp(ros::NodeHandle &node_handle);
  ~URImp();

  // Callbacks
//  bool srvCommandCb(impact_interface::InterfaceCmd::Request & req,
//                    impact_interface::InterfaceCmd::Response &res);
  void getRobotJPosCb(const sensor_msgs::JointState::ConstPtr &msg);
  void getOmegaPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void getForceCb(const geometry_msgs::WrenchStamped::ConstPtr &msg);
//  void getRobotTPoseCb(const std_msgs::Float64MultiArray::ConstPtr &msg);

  // Functions

  void control_loop();
//  int getIK();

//  int stop();
//  int use2Dof();
//  int use4Dof();
//  int getStatus();
//  int getPoseFK();
//  int sendPoseCmd();
//  int getDI(int input);

private:
  ros::NodeHandle nh_;

  // Service Server
//  ros::ServiceServer srv_cmd_;

  // Subscribers
  ros::Subscriber sub_robot_pose_;
  ros::Subscriber sub_omega_pose_;
  ros::Subscriber sub_force_;
//  ros::Subscriber sub_t_pose_des_;

  // Publisher
  ros::Publisher pub_robot_tar_pose_;
//  ros::Publisher pub_joint_pos_;
//  ros::Publisher pub_target_tip_pos_;
//  ros::Publisher pub_target_tip_ori_;

  // Publisher Msgs
  geometry_msgs::PoseStamped pub_robot_tar_pose_msg_;
//  std_msgs::Float64MultiArray pub_joint_pos_msg_;
//  std_msgs::Float64MultiArray pub_target_tip_pos_msg_;
//  std_msgs::Float64MultiArray pub_target_tip_ori_msg_;

  // Augmented Dual Force control parameters -> Admittance Control
  Eigen::Matrix3d K_trans_; /* Wandering: Stifness for Translation motion */
  Eigen::Matrix3d K_rot_;   /* Wandering: Stifness for Rotation motion */
  Eigen::Matrix3d C_trans_; /* Wandering: Damper for Translation motion */
  Eigen::Matrix3d C_rot_;   /* Wandering: Damper for Rotation motion */
  Eigen::Matrix3d M_trans_; /* Wandering: Inertia for Translation motion */
  Eigen::Matrix3d M_rot_;   /* Wandering: Inertia for Rotation motion */
  Eigen::Vector3d V_d_;     /* Wandering: Actual trans. velocity: V[k] */
  Eigen::Vector3d V_temp_;  /* Wandering: Previous velocity: V[k-1] */
  Eigen::Vector3d w_d_;     /* Wandering: Actual angular velocity: w[k] */
  Eigen::Vector3d w_temp_;  /* Wandering: Previous angular velocity: w[k-1] */

  //Local variables
  Eigen::Vector3d force_;
  Eigen::Vector3d torque_;
  Eigen::VectorXd robot_j_pos_;
  Eigen::Vector3d robot_tar_cart_pos_;
  Eigen::VectorXd omega_pose_;
  Eigen::Vector3d imp_delta_pos_;

  //Flag
  bool flag_omega_rcv_;

//  std::string name_;
//  VectorXi    di_status_;
//  VectorXd    ai_status_;
//  VectorXd    ai_status1_;

//  // Filters
//  filters::IIR *filter_iir_pos;
//  filters::SMA *filter_sma_ori;

//  // Robot Frames
//  bool flag_robot_initialized_;

//  Eigen::Matrix3d Rd_0_1_;
//  Eigen::Vector3d Td_0_1_;
//  Eigen::Matrix3d R_1_5_;
//  Eigen::Vector3d P15_1_;

//  // Interface variables
//  int                      i_num_;
//  int                      i_status_;
//  std::string              i_type_;
//  int                      num_joints_;
//  std::vector<std::string> joint_names_;

//  Eigen::VectorXd T_0_6_;

//  Eigen::VectorXd joint_position_;
//  Eigen::VectorXd ee_euler_pose_;
//  Eigen::VectorXd target_tip_pose_;
//  Eigen::VectorXd target_tip_pos_;
//  Eigen::VectorXd target_tip_ori_n_;
//  Eigen::VectorXd target_tip_ori_o_;
//  Eigen::VectorXd target_tip_ori_a_;

//  Eigen::VectorXd joint_limits_min_;
//  Eigen::VectorXd joint_limits_max_;
//  Eigen::VectorXd joint_range_;
//  Eigen::VectorXd sensor_range_;
//  Eigen::VectorXd sensor_offset_;
//  Eigen::VectorXd sensor_resol_;
//  Eigen::VectorXi ai_port_;

//  float  rot_volt_offset_0_;
//  float  rot_volt_offset_0_prev_;
//  float  rot_volt_offset_1_;
//  float  rot_volt_offset_1_prev_;
//  double rot_angle_;
//  double rot_angle_prev_;

}; // class Interface

} // namespace nu_interface

#endif
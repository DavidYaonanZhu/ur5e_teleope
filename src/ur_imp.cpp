#include "ur_imp/ur_imp.h"

namespace ur_nu
{

    URImp::URImp(ros::NodeHandle &node_handle){

        nh_ = node_handle;

        sub_robot_pose_ =
            nh_.subscribe("/joint_states", 1, &URImp::getRobotJPosCb, this);
        sub_omega_pose_ =
            nh_.subscribe("/sigma7/sigma0/pose", 1, &URImp::getOmegaPoseCb, this);
        sub_force_ =
            nh_.subscribe("/sigma7/sigma0/force_filtered", 1, &URImp::getForceCb, this);

        pub_robot_tar_pose_ =
            nh_.advertise<geometry_msgs::PoseStamped>("ur_imp/robot_tar_pose", 1);
        pub_dx_pos_ =
            nh_.advertise<std_msgs::Float64MultiArray>("ur_imp/dx", 1);


        pub_dx_pos_msg_.data.resize(3,0);

        // Virtual Stiffness
        K_trans_ = Matrix3d::Identity();
        K_trans_(0, 0) = 0; //
        K_trans_(1, 1) = 0; //
        K_trans_(2, 2) = 0; //

        // Virtual Damping
        C_trans_ = Matrix3d::Identity();
        C_trans_(0, 0) = 0; // Wacoh: 80 ATI17: 35 ATI40: 50 ATI40RobForceps:  150/400
        C_trans_(1, 1) = 0; // Wacoh: 80 ATI17: 35 ATI40: 50 ATI40RobForceps:   150/80
        C_trans_(2, 2) = 200; // Wacoh: 80 ATI17: 60 ATI40: 50 ATI40RobForceps:  150/400

        C_rot_ = Matrix3d::Identity();
        C_rot_(0, 0) = 0.5; // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps:  0.7/0.5
        C_rot_(1, 1) = 0.5; // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps:  1.0/0.8
        C_rot_(2, 2) = 0.5; // Wacoh: 2 ATI17:0.3 ATI40: 0.8 ATI40RobForceps:  1.0/0.5

        // Virtual Mass
        M_trans_ = Matrix3d::Identity();
        M_trans_(0, 0) = 0; // Wacoh 2 ATI17: 6 ATI40: 4.5 ATI40RobForceps: 4.5/8
        M_trans_(1, 1) = 0; // Wacoh 2 ATI17: 6 ATI40: 4.5 ATI40RobForceps: 4.5/8
        M_trans_(2, 2) = 0.1; // Wacoh 2 ATI17: 8 ATI40: 4.5 ATI40RobForceps: 4.5/8

        M_rot_ = Matrix3d::Identity();
        M_rot_(0, 0) = 0.02; // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02
        M_rot_(1, 1) = 0.02; // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02
        M_rot_(2, 2) = 0.02; // Wacoh: 0.5 ATI17:0.5 ATI40: 0.07 ATI40RobForceps:  0.07/0.02

        //Local variables
        robot_j_pos_ = VectorXd::Zero(6);
        omega_pose_ = VectorXd::Zero(7);
        force_ = Vector3d::Zero();
        torque_ = Vector3d::Zero();
        robot_tar_cart_pos_ = Vector3d::Zero();

        flag_omega_rcv_ = false;
    }

    //Callbacks
    void URImp::getForceCb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        geometry_msgs::Wrench wrench_tmp;
        wrench_tmp = msg->wrench;
      force_[0] = wrench_tmp.force.x;
      force_[1] = wrench_tmp.force.y;
      force_[2] = wrench_tmp.force.z;
      torque_[0] = wrench_tmp.torque.x;
      torque_[1] = wrench_tmp.torque.y;
      torque_[2] = wrench_tmp.torque.z;
    }
    URImp::~URImp(){}


    void URImp::getRobotJPosCb(const sensor_msgs::JointState::ConstPtr &msg)
    {
        robot_j_pos_ = VectorXd::Map(&msg->position[0], msg->position.size());
    }

    void URImp::getOmegaPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        geometry_msgs::Pose pose_tmp;
        pose_tmp = msg->pose;
        geometry_msgs::Point position_tmp;
        position_tmp = pose_tmp.position;
        geometry_msgs::Quaternion orientation_tmp;
        orientation_tmp = pose_tmp.orientation;

        omega_pose_[0] = position_tmp.x;
        omega_pose_[1] = position_tmp.y;
        omega_pose_[2] = position_tmp.z;
        omega_pose_[3] = orientation_tmp.x;
        omega_pose_[4] = orientation_tmp.y;
        omega_pose_[5] = orientation_tmp.z;
        omega_pose_[6] = orientation_tmp.w;

        flag_omega_rcv_ = true;
    }

    void URImp::control_loop(){

        if (flag_omega_rcv_){
            // TRANSLATION
            int i;
            for (i = 2; i < 3; i++) {
              V_d_[i] = (M_trans_(i, i) * V_temp_(i) + thread_sampling_time_sec_d_ * force_[i]) /
                          (thread_sampling_time_sec_d_ * C_trans_(i, i) + M_trans_(i, i)); // (m/s)
            }
            //    V_d_[2]=0.0; //ATI-> Problem with force Z when torque is applied
            if (V_d_.norm() >= VEL_LIMIT_WAND)
              V_d_ = V_d_ * (VEL_LIMIT_WAND / V_d_.norm()); // Translation velocity software limit
            ROS_INFO_STREAM("Vd (mm/s): " << 1000 * V_d_.transpose());

            V_temp_ = V_d_; /* v[k-1] = v[k] */
            //    Td_0_3_ = Td_0_3_ + 1000*thread_sampling_time_sec_d_*V_d_1_; // (mm)
            imp_delta_pos_ = imp_delta_pos_ + thread_sampling_time_sec_d_ * V_d_; // (m)
            ROS_INFO_STREAM("delta_x (mm/s):" << 1000*imp_delta_pos_.transpose());



            robot_tar_cart_pos_[0] = omega_pose_[0] + imp_delta_pos_[0];
            robot_tar_cart_pos_[1] = omega_pose_[1] + imp_delta_pos_[1];
            robot_tar_cart_pos_[2] = omega_pose_[2] + imp_delta_pos_[2];


            geometry_msgs::Point position_tmp;
            geometry_msgs::Quaternion orientation_tmp;
            std_msgs::Header header_tmp;


            position_tmp.x = robot_tar_cart_pos_[0];
            position_tmp.y = robot_tar_cart_pos_[1];
            position_tmp.z = robot_tar_cart_pos_[2];

            orientation_tmp.x = omega_pose_[3];
            orientation_tmp.y = omega_pose_[4];
            orientation_tmp.z = omega_pose_[5];
            orientation_tmp.w = omega_pose_[6];


            geometry_msgs::Pose pose_tmp;
            pose_tmp.position = position_tmp;
            pose_tmp.orientation = orientation_tmp;
            header_tmp.stamp = ros::Time::now();


            pub_robot_tar_pose_msg_.header = header_tmp;
            pub_robot_tar_pose_msg_.pose = pose_tmp;

            pub_dx_pos_msg_.data[0] = imp_delta_pos_[0];
            pub_dx_pos_msg_.data[1] = imp_delta_pos_[1];
            pub_dx_pos_msg_.data[2] = imp_delta_pos_[2];
            pub_dx_pos_.publish(pub_dx_pos_msg_);

            pub_robot_tar_pose_.publish(pub_robot_tar_pose_msg_);
        }



    }

}

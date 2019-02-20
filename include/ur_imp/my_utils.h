/**********************************************************************************
    my_print_utils.h

    header file for my_print_utils.cpp

    October 2015, Jun Nakanishi
**********************************************************************************/
#ifndef MY_UTILS_H
#define MY_UTILS_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

namespace MyUtils{
//  convert std::vector to Eigen::Vector
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> stdVec2EigenVec(std::vector<T> &vec){

    return Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, 1> >(&vec[0], vec.size());

}

//  print elements of Eigen quaternion
template <typename T>
void printEigenQuaternion (Eigen::Quaternion<T>  &quat)
{
    //    ROS_INFO_STREAM("quat(w,x,y,z) " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z());
    ROS_INFO_STREAM("quat(x,y,z,w) " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w());
}

//  print elements of vector
template <typename T>
void print_vec (T &vec)
{
    ROS_INFO_STREAM(stdVec2EigenVec(vec).transpose());
}

//  convert Eigen::Vector to std::vector
template <typename T>
std::vector<T> EigenVec2StdVec(Eigen::Matrix<T, Eigen::Dynamic, 1> &eigen_vec){

    std::vector<T> vec;
    vec.resize(eigen_vec.rows());

    Eigen::Map< Eigen::Matrix<T, Eigen::Dynamic, 1> >(&vec[0], eigen_vec.rows(), 1) = eigen_vec;

    return vec;
}

// convert quaternion to RPY
Eigen::Vector3d Quat2RPY(const Eigen::Quaterniond q);

// calculate mininum jerk (5-th order polynomial) next step
bool calculate_min_jerk_next_step (double &x, double &xd, double &xdd,
                                            double des_x,
                                            double tau,
                                            double delta_t);

bool calculate_min_jerk_next_step (Eigen::VectorXd &x, Eigen::VectorXd &xd, Eigen::VectorXd &xdd,
                                            Eigen::VectorXd des_x,
                                            double tau,
                                            double delta_t);



double limit_val(double val, double val_max);        // impose min/max for input
double deadzone_val(double y, double LL, double UL); // deadzone

}

#endif // MY_UTILS_H

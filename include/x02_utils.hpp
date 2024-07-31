#ifndef X02_UTILS_HPP
#define X02_UTILS_HPP
// #include "pinocchio/multibody/fcl.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <string>
#include <iostream>
#include <algorithm>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
// #include <utils/proxs.hpp>



typedef Eigen::Matrix<double, 6, 1> Vector6d;

class X02_Utils 
{
public:
    std::shared_ptr<pinocchio::SE3> M_des,M_L, M_R;
    std::unique_ptr<Eigen::VectorXd> q,q_sol_l,q_sol_r;
    std::unique_ptr<Eigen::VectorXd> v_l,q_l,v_r,q_r;
    bool success_l, success_r;
    std::shared_ptr<Vector6d> err_l,err_r;
    double eps,DT,damp;
    int IT_MAX;
    X02_Utils(std::string urdf_path);
    ~X02_Utils();
    bool ComputeLeftHandIK(Eigen::Matrix3d rotm, Eigen::Vector3d pos,Eigen::VectorXd q_get);
    bool ComputeRightHandIK(Eigen::Matrix3d rotm, Eigen::Vector3d pos,Eigen::VectorXd q_get);
    void ComputeLeftHandFK(Eigen::VectorXd q_get);
    void ComputeRightHandFK(Eigen::VectorXd q_get);
    void getJntStates(Eigen::VectorXd q_get);
    double mapToRange(double x);
    Eigen::VectorXd mapVecToRange(Eigen::VectorXd& vec);
    Eigen::Vector3d Mat2RPY(Eigen::Matrix3d rotm);
    Eigen::Vector3d matToRpy(Eigen::Matrix3d rotm);
    // Eigen::Matrix3d RPY2RotMat(Eigen::Vector3d rpy, Eigen::Vector3d rpy_source);
    Eigen::Matrix3d IdenMat();
    Eigen::Vector3d Stick2RobotLeftPos(Eigen::Vector3d pos, Eigen::Vector3d pos_source);
    Eigen::Vector3d Stick2RobotRightPos(Eigen::Vector3d pos, Eigen::Vector3d pos_source);
    Eigen::Matrix3d Stick2RobotLeftRPY(Eigen::Vector3d rpy, Eigen::Vector3d rpy_source);
    Eigen::Matrix3d Stick2RobotRightRPY(Eigen::Vector3d rpy, Eigen::Vector3d rpy_source);

private:
    std::unique_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::Data> data;
    std::shared_ptr<pinocchio::Data::Matrix6x> J_l,J_r;
    
};

#endif //X02_UTILS_HPP
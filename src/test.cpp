#include "x02_utils.hpp"
#include "lcm_gen/vr_lcm_recv.hpp"
#include <iostream>



int main(int argc, char** argv)
{
     typedef Eigen::Matrix<double, 12, 1> Vector12d;
   // typedef Eigen::Matrix<double, 14, 1> Vector14d;
    
    std::string urdf_path = "/home/toland/Codefield/X02_Lite/models/X02_Lite/X02lite_V2.urdf";
    std::shared_ptr<X02_Utils> utils;
    utils = std::make_shared<X02_Utils>(urdf_path);
    Vector12d sim_joint_states;
    sim_joint_states[0] = 0.0;
    sim_joint_states[1] = 0.0;
    sim_joint_states[2] = 0.0;
    sim_joint_states[3] = 1.5;
    sim_joint_states[4] = 0.0;
    sim_joint_states[5] = 0.0;
    sim_joint_states[6] = 0.0;

    sim_joint_states[7] = 0.0;
    sim_joint_states[8] = 0.0;
    sim_joint_states[9] = 0.0;
    sim_joint_states[10] = 1.5;
    sim_joint_states[11] = 0.0;
    // sim_joint_states[12] = 0.0;
    // sim_joint_states[13] = 0.0;
    utils->ComputeLeftHandFK(sim_joint_states);
    std::cout << "******\n" << std::endl;
    Eigen::Matrix3d rot_l = utils->M_L->rotation();
    Eigen::Vector3d left_hand_rpy_1 = utils->Mat2RPY(rot_l);
    Eigen::Vector3d left_hand_rpy_2 = utils->matToRpy(rot_l);
    utils->ComputeRightHandFK(sim_joint_states);
    Eigen::Matrix3d rot_r = utils->M_R->rotation();
    Eigen::Vector3d right_hand_rpy_1 = utils->Mat2RPY(rot_r);
    Eigen::Vector3d right_hand_rpy_2 = utils->matToRpy(rot_r);
    std::cout <<  "   left_hand_rpy_1===== \n" << left_hand_rpy_1 << std::endl;
    std::cout <<  "   right_hand_rpy_1===== \n" << right_hand_rpy_1 << std::endl;
    printf("\n");
    std::cout <<  "   left_hand_rpy_2===== \n" << left_hand_rpy_2 << std::endl;
    std::cout <<  "   right_hand_rpy_2===== \n" << right_hand_rpy_2 << std::endl;


    // Eigen::Vector3d target_pos;
    // Eigen::Matrix3d target_mat;
    // target_pos = utils.M_fk->translation();
    // target_mat = utils.M_fk->rotation();
    // std::cout << "target_pos===" << target_pos << std::endl;
    // std::cout << "target_mat===" << target_mat << std::endl;

    
    
    return 0;
}
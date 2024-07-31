#include "x02_utils.hpp"
#include <cmath>


X02_Utils::X02_Utils(std::string urdf_path){
      model = std::make_unique<pinocchio::Model>();
      pinocchio::urdf::buildModel(urdf_path, *model);
      data = std::make_shared<pinocchio::Data>(*model);
      J_l = std::make_shared<pinocchio::Data::Matrix6x>(6,model->nv);
      J_r = std::make_shared<pinocchio::Data::Matrix6x>(6,model->nv);
      M_des = std::make_shared<pinocchio::SE3>(Eigen::Matrix3d::Identity(),
                                               Eigen::Vector3d(1., 0., 1.));
      q = std::make_unique<Eigen::VectorXd>(model->nv);
      v_l = std::make_unique<Eigen::VectorXd>(model->nv);
      q_l = std::make_unique<Eigen::VectorXd>(model->nv);
      q_sol_l = std::make_unique<Eigen::VectorXd>(6);
      v_r = std::make_unique<Eigen::VectorXd>(model->nv);
      q_r = std::make_unique<Eigen::VectorXd>(model->nv);
      q_sol_r = std::make_unique<Eigen::VectorXd>(6);
      err_l = std::make_shared<Vector6d>();
      err_r = std::make_shared<Vector6d>();
      Eigen::VectorXd ulim(model->nv),llim(model->nv);
      Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
      Eigen::Vector3d v =  Eigen::Vector3d::Zero();
      M_L = std::make_shared<pinocchio::SE3>(m,v);
      M_R = std::make_shared<pinocchio::SE3>(m,v);
      success_l = false;
      success_r = false;
      eps = 1e-1;
      IT_MAX = 300;
      DT = 2e-1;
      damp = 1e-12;
      
}

X02_Utils::~X02_Utils(){
     std::cout<< "X02_Utils finished" <<std::endl;
}

double X02_Utils::mapToRange(double x) {

    double mapped = fmod(x, 2 * M_PI);
    
    if (mapped > M_PI) {
        mapped -= 2 * M_PI;
    }
    
    return mapped;
}

Eigen::VectorXd X02_Utils::mapVecToRange(Eigen::VectorXd& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        vec(i) = mapToRange(vec(i));
    }
    return vec;
}


void X02_Utils::getJntStates(Eigen::VectorXd q_get){
    *q = q_get;
}

bool X02_Utils::ComputeLeftHandIK(Eigen::Matrix3d rotm, Eigen::Vector3d pos,Eigen::VectorXd q_get){
    //  std::cout << "L_wrist=====" << model->getJointId("L_Wrist") << std:: endl;
    //  std::cout << "R_wrist=====" << model->getJointId("R_Wrist") << std:: endl;
     M_des = std::make_shared<pinocchio::SE3>(rotm, pos);    
     J_l->setZero();
     *q_l = q_get;
     for (int i = 0;; i++)
     {
        pinocchio::forwardKinematics(*model, *data, *q_l);
        const pinocchio::SE3 iMd = data->oMi[model->getJointId("L_Wrist2")].actInv(*M_des);
        *err_l = pinocchio::log6(iMd).toVector(); // in joint frame
        if (err_l->norm() < eps)
        {
          success_l = true;
          break;
        }
        if (i >= IT_MAX)
        {
          success_l = false;
          break;
        }
        pinocchio::computeJointJacobian(*model, *data, *q_l, model->getJointId("L_Wrist2"), *J_l); // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        *J_l = -Jlog * (*J_l);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = (*J_l) * J_l->transpose();
        JJt.diagonal().array() += damp;
        v_l->noalias() = -J_l->transpose() * JJt.ldlt().solve(*err_l);
        *q_l = pinocchio::integrate(*model, *q_l, (*v_l) * DT);
      }
  
      if (success_l)
      {
        std::cout << "Left Convergence achieved!" << std::endl;
        for(int i=0;i<6;i++){
          (*q_sol_l)[i] = mapToRange((*q_l)[i]);
        }
        return true;
      }
      else
      {
        // std::cout
        //   << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
        //   << std::endl;
        for(int i=0;i<6;i++){
          (*q_sol_l)[i] = mapToRange(q_get[i]);
        }
        return false;
      }
    // printf("left--left");
    // return false;
 }






bool X02_Utils::ComputeRightHandIK(Eigen::Matrix3d rotm, Eigen::Vector3d pos,Eigen::VectorXd q_get){
     M_des = std::make_shared<pinocchio::SE3>(rotm, pos);    
     J_r->setZero();
     *q_r = q_get;
     for (int i = 0;; i++)
     {
        pinocchio::forwardKinematics(*model, *data, *q_r);
        const pinocchio::SE3 iMd = data->oMi[model->getJointId("R_Wrist2")].actInv(*M_des);
        *err_r = pinocchio::log6(iMd).toVector(); // in joint frame
        if (err_r->norm() < eps)
        {
          success_r = true;
          break;
        }
        if (i >= IT_MAX)
        {
          success_r = false;
          break;
        }
        pinocchio::computeJointJacobian(*model, *data, *q_r, model->getJointId("R_Wrist2"), *J_r); // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        *J_r = -Jlog * (*J_r);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = (*J_r) * J_r->transpose();
        JJt.diagonal().array() += damp;
        v_r->noalias() = -J_r->transpose() * JJt.ldlt().solve(*err_r);
        *q_r = pinocchio::integrate(*model, *q_r, (*v_r) * DT);

      }
  
      if (success_r)
      {
        std::cout << "Right Convergence achieved!" << std::endl;
        for(int i=6;i<12;i++){
          std::cout<< "(*q_sol_r)[ " << i-6 <<" ]" << (*q_r)[i] << std::endl; 
          (*q_sol_r)[i-6] = mapToRange((*q_r)[i]);
        }
        return true;
      }
      else
      {
        // std::cout
        //   << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
        //   << std::endl;
        for(int i=6;i<12;i++){
          (*q_sol_r)[i-6] = mapToRange(q_get[i]);
        }
        return false;
      }
    // return false;
 }


void X02_Utils::ComputeLeftHandFK(Eigen::VectorXd q_get){
     pinocchio::forwardKinematics(*model, *data, q_get);
     pinocchio::SE3 OMI = data->oMi[model->getJointId("L_Wrist")];
     Eigen::Matrix3d m = OMI.rotation();
     Eigen::Vector3d v = OMI.translation();
     M_L->translation() = v;
     M_L->rotation() = m;
     std::cout << "M_L==\n" << *M_L << std::endl;
}

void X02_Utils::ComputeRightHandFK(Eigen::VectorXd q_get){
     pinocchio::forwardKinematics(*model, *data, q_get);
     pinocchio::SE3 OMI = data->oMi[model->getJointId("R_Wrist")];
     Eigen::Matrix3d m = OMI.rotation();
     Eigen::Vector3d v = OMI.translation();
     M_R->translation() = v;
     M_R->rotation() = m;
     std::cout << "M_R==\n" << *M_R << std::endl;
}


Eigen::Vector3d X02_Utils::Mat2RPY(Eigen::Matrix3d rotm){
    Eigen::Vector3d rpy = rotm.eulerAngles(2,1,0);
    return rpy;
}

Eigen::Vector3d X02_Utils::matToRpy(Eigen::Matrix3d rotm){
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(rotm);
    // Eigen::Vector3d rpy = rotm.eulerAngles(2,1,0);
    return rpy;
}


Eigen::Matrix3d X02_Utils::IdenMat(){
    Eigen::Matrix3d rotm_cmd = Eigen::Matrix3d::Identity();
    return rotm_cmd;
}



Eigen::Matrix3d X02_Utils::Stick2RobotLeftRPY(Eigen::Vector3d rpy, Eigen::Vector3d rpy_source){
     Eigen::Matrix3d rotm_cmd;
     Eigen::Vector3d rpy_cmd;
     rpy_cmd[0]=rpy_source[0] + rpy[2];
     rpy_cmd[1]=rpy_source[1] + rpy[1];
     rpy_cmd[2]=rpy_source[2] ;//- rpy[0];
     rotm_cmd = Eigen::AngleAxisd(rpy_cmd[0], Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(rpy_cmd[1], Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(rpy_cmd[2], Eigen::Vector3d::UnitX());
     return rotm_cmd;
}


Eigen::Matrix3d X02_Utils::Stick2RobotRightRPY(Eigen::Vector3d rpy, Eigen::Vector3d rpy_source){
     Eigen::Matrix3d rotm_cmd;
     Eigen::Vector3d rpy_cmd;
     rpy_cmd[0]=rpy_source[0] + rpy[2];
     rpy_cmd[1]=rpy_source[1] - rpy[1];
     rpy_cmd[2]=rpy_source[2] ;//+ rpy[0];
     rotm_cmd = Eigen::AngleAxisd(rpy_cmd[0], Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(rpy_cmd[1], Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(rpy_cmd[2], Eigen::Vector3d::UnitX());
     return rotm_cmd;
}

Eigen::Vector3d X02_Utils::Stick2RobotLeftPos(Eigen::Vector3d pos, Eigen::Vector3d pos_source){
     Eigen::Vector3d pos_cmd;
     pos_cmd[0] = pos_source[0] - pos[2];
     pos_cmd[1] = pos_source[1] - pos[0];
     pos_cmd[2] = pos_source[2] + pos[1];
     return pos_cmd;
}

Eigen::Vector3d X02_Utils::Stick2RobotRightPos(Eigen::Vector3d pos, Eigen::Vector3d pos_source){
     Eigen::Vector3d pos_cmd;
     pos_cmd[0] = pos_source[0] - pos[2];
     pos_cmd[1] = pos_source[1] - pos[0];
     pos_cmd[2] = pos_source[2] + pos[1];
     return pos_cmd;
}

// int main(int /* argc */, char ** /* argv */)
// {
//    std::string urdf_path="/home/leeezd/X02Lite/models/X02_2/X02_2_WB.urdf";
//    X02_Utils X02(urdf_path); 
//    Eigen::Matrix3d rotm;
//    Eigen::Vector3d pos;

//    Eigen::VectorXd q_current(16);
//    for (int i=0;i<16;i++){
//     q_current[i] = 0.5;
//    }
//    q_current[0]=0;
//    q_current[15]=0;
//    X02.getJntStates(q_current);
//    pos << 0.16,-0.089,1.65;
//    rotm << 0.35,-0.60,0.71,
//            0.04,0.77,0.63,
//            -0.93,-0.19,0.30;  
   
//    X02.ComputeLeftHandIK(rotm,pos,q_current); 
//    X02.ComputeLeftHandFK(*(X02.q_l)); 

//    return 0;                              
// }
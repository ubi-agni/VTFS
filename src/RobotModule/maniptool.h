#ifndef MANIPTOOL_H
#define MANIPTOOL_H

#include "UtilModule/RebaType.h"
#include <vector>
#include "UtilModule/Util.h"
#include "TacModule/myrmextac.h"
#include "RobotModule/RobotState.h"

#include <fstream>
#include <iostream>

class RobotState;

struct ToolState {
    //default case, dof_num = 0
  int dof_num;
  //default case, there is no pose in the vector
  std::vector<RG_Pose> eef_pose;
  //desribe the initialzed orientation guess
  //transform  from tactool to robot eef
  //Ttool = Teef * rel_eef_tactool;
  Eigen::Matrix3d rel_o;
  //rotation matrix from tactile sensor frame to arbitary tactile sensor tangent frame
  Eigen::Matrix3d rotate_s2sdot;
  //estimated sensor frame after the update.
  Eigen::Matrix3d tac_sensor_cfm_local;

  double cur_ctc_x;
  double cur_ctc_y;
  double init_ctc_x;
  double init_ctc_y;
  ToolState()
      {
          dof_num = 0;
          init_ctc_x = -1;
          init_ctc_y = -1;
          cur_ctc_x = -1;
          cur_ctc_y = -1;
          rel_o.setIdentity();
          rotate_s2sdot.setIdentity();
          tac_sensor_cfm_local.setIdentity();
      }
} ;

class ManipTool
{
public:
    ManipTool(RobotState *);
    //default case, mtt = Notool
    ManipuToolT mtt;
    ToolState ts;
    Eigen::Vector3d update_translation_est(Eigen::Vector3d lv,Eigen::Vector3d rv,\
                                 Eigen::Matrix3d robot_eef_rm, MyrmexTac *myrtac);
    void update_nv(Eigen::Vector3d lv,Eigen::Vector3d &n_hat,Eigen::Vector3d& nv_dot_fb);
    //parameters required for the translation learning
    Eigen::Matrix3d Gama_r;
    Eigen::Matrix3d L_r;
    Eigen::Matrix3d L_r_dot;
    Eigen::Vector3d  c_r;
    Eigen::Vector3d  c_r_dot;
    double beta_r;

    //parameters for the normal direction estimation
    Eigen::Matrix3d P_bar;
    double Gama_n = 2;
    Eigen::Matrix3d L_n;
    Eigen::Matrix3d L_n_dot;
    Eigen::Vector3d nv_dot;
    double beta_n = 0.99;

    Eigen::Vector3d est_trans;
    Eigen::Vector3d est_trans_dot;
    Eigen::Vector3d est_nv;
    void update_tac_sensor_cfm_local();
    void store_parameters(std::string fn_nv, std::string fn_rorate,std::string fn_trans);
    void load_parameters(std::string fn_nv, std::string fn_rorate,std::string fn_trans);
    std::fstream f_nv,f_rotate,f_trans;
};

#endif // MANIPTOOL_H

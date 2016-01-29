#ifndef MANIPTOOL_H
#define MANIPTOOL_H

#include "UtilModule/RebaType.h"
#include <vector>



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
      }
} ;

class ManipTool
{
public:
    ManipTool();
    //default case, mtt = Notool
    ManipuToolT mtt;
    ToolState ts;
};

#endif // MANIPTOOL_H

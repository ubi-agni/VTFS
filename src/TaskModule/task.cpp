#include "task.h"

#define initP_x 0.28
#define initP_y 0.3
#define initP_z 0.30

#define initO_x 0.0
#define initO_y M_PI/2;
#define initO_z 0.0;

Task::Task()
{
    desired_p_eigen(0) = -1 * initP_x;
    desired_p_eigen(1) = initP_y;
    desired_p_eigen(2) = initP_z;
    desired_o_ax(0) = initO_x;
    desired_o_ax(1) = initO_y;
    desired_o_ax(2) = initO_z;
    curtaskname.tact = CONTACT_FORCE_TRACKING;
    curtaskname.prot = RP_NOCONTROL;
    curtaskname.vist = NV_VERT_ALIGN;
    desired_surf_nv.setZero();
    desired_axis_dir.setZero();
    contact_frame.setIdentity();
    //modality type
    mt = JOINTS;
    //motion frame type
    mft = GLOBAL;
    //exploration mode type
    emt = NOEXPLORE;
    velocity_p2p.setZero();
    desired_dis = 4;
}

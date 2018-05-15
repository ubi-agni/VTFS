#ifndef TASK_H
#define TASK_H
#include <Eigen/Dense>
#include <vector>

enum TacFBType{
    TAXEL_NUM = 0,
    TAXEL_POSITION
};

enum TACTaskNameT{
    CONTACT_POINT_TRACKING,
    CONTACT_POINT_FORCE_TRACKING,
    CONTACT_FORCE_TRACKING,
    SENSING_POLE_TRACKING,
    Z_ORIEN_TRACKING,
    LINEAR_TRACKING,
    COVER_OBJECT_SURFACE,
    MOVETO_DEFINED_POINT,
    OBJECT_SURFACE_EXPLORING,
    SINOID_FORCE_TRACKING,
    ORIEN_FOLLOWING,
    ORIEN_ALIGNMENT,
    GUIDE_CONTACT,
    OPENLOOP_FORWARD,
    OPENLOOP_BACKWARD,
    T_NOCONTROL,
    LEARN_TACTOOL_CONTACT,
    LEARN_TACTOOL_SLIDING,
    LEARN_TACTOOL_ROLLING
};


enum PROTaskNameT{
    RLXP = 0,
    RLYP = 1,
    RLZP = 2,
    RRXP = 3,
    RRYP = 4,
    RRZP = 5,
    RLXN = 7,
    RLYN = 8,
    RLZN = 9,
    RRXN = 10,
    RRYN = 11,
    RRZN = 12,
    RP_NOCONTROL = 13,
    RP_LINEFOLLOW = 14,
    RP_ROTATEFOLLOW = 15,
    RP_BOTHFOLLOW
};

enum VISTaskNameT{
    V_NOCONTROL = 0,
    NV_VERT_ALIGN
};

enum FORCETaskNameT{
    F_MAINTAIN = 0,
    F_CURVETRACKING
};

struct TaskNameT{
   TACTaskNameT tact;
   PROTaskNameT prot;
   VISTaskNameT vist;
   FORCETaskNameT forcet;
};

enum ModalityT{
    JOINTS,
    VISION,
    VISION3D,
    TACTILE,
    FORCE
};

enum MoveFrameT{
    GLOBAL,
    LOCAL,
    LOCALP2P
};

//explore action mode for tactool learning
enum ExploreModeT{
    NOEXPLORE,
    LINEAREXPLORE,
    ROTATEEXPLORE
};

class Task
{
public:
    Task();
    virtual Eigen::Vector3d get_desired_p_eigen() = 0;
    virtual Eigen::Vector3d get_initial_p_eigen() = 0;
    virtual Eigen::Matrix3d get_desired_o_eigen() = 0;
    virtual Eigen::Vector3d get_desired_o_ax() = 0;
    //specify the desired robot movement distance
    virtual double get_desired_mv_dis() = 0;
    virtual void set_desired_mv_dis(double s) = 0;
    virtual void set_desired_p_eigen(Eigen::Vector3d p) = 0;
    virtual void set_initial_p_eigen(Eigen::Vector3d p) = 0;
    virtual void set_desired_o_eigen(Eigen::Matrix3d o_eigen) = 0;
    virtual void set_desired_o_ax(Eigen::Vector3d o_ax) = 0;
    virtual void set_desired_axis_dir(Eigen::Vector3d ax_dir) = 0;
    virtual void set_desired_surf_nv(Eigen::Vector3d surf_nv) = 0;
    virtual void set_desired_cp_myrmex(double *) = 0;
    virtual void set_desired_cf_myrmex(double) = 0;
    virtual void set_desired_cf_kuka(double) = 0;
    virtual void set_desired_taxel_mid(int) = 0;
    virtual void set_contact_frame(Eigen::Matrix3d) = 0;
    virtual void get_desired_position_mid(Eigen::Vector3d &)=0;
    virtual void set_desired_position_mid(Eigen::Vector3d) = 0;
    virtual void get_desired_nv_mid(Eigen::Vector3d &)=0;
    virtual void set_desired_nv_mid(Eigen::Vector3d) = 0;
    virtual void set_desired_cf_mid(double) = 0;
    virtual void set_taxelfb_type_mid(TacFBType type) = 0;
    virtual void set_desired_cp_moving_dir(double x, double y) = 0;
    virtual void set_desired_rotation_range(double,double,double) = 0;
    virtual void set_desired_orien_myrmex(double) = 0;
    virtual Eigen::Vector3d get_desired_rotation_range() = 0;
    TaskNameT curtaskname;
    ModalityT mt;
    MoveFrameT mft;
    ExploreModeT emt;
    Eigen::Vector3d velocity_p2p;
    TacFBType tft;
    Eigen::Vector3d desired_pose_range;
    double desired_dis;
protected:
    Eigen::Vector3d desired_p_eigen,initial_p_eigen;
    Eigen::Matrix3d desired_o_eigen;
    Eigen::Vector3d desired_o_ax;
public:
    Eigen::Vector3d desired_axis_dir;
    Eigen::Vector3d desired_surf_nv;
    Eigen::Matrix3d contact_frame;
};

#endif // TASK_H

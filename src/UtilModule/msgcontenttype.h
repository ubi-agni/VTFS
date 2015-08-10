#ifndef MSGCONTENTTYPE_H
#define MSGCONTENTTYPE_H
struct kuka_msg {
  Eigen::Vector3d p;
  Eigen::Matrix3d o;
  Eigen::VectorXd ft;
};

struct tacpcs_msg{
    Eigen::Vector3d position3d;
    Eigen::Vector3d CPnormal;
    Eigen::Vector3d position3d_Object;
    Eigen::Vector3d CPnormal_Object;
};

struct myrmex_msg {
    double cogx;
    double cogy;
    int contactnum;
    bool contactflag;
    double cf;
    double lineorien;
};

struct markered_object_msg{
    Eigen::Vector3d p;
    Eigen::Matrix3d orientation;
    double roll,pitch,yaw;
    int marker_num;
};

struct dirt_points_msg{
    int markerpoints_num;
    Eigen::Vector3d markerpoints_pos;
    Eigen::Vector3d markerpoints_nv;
};

#endif // MSGCONTENTTYPE_H

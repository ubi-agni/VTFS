#include "RobotState.h"
#include "UtilModule/Util.h"

#define m_cog_cent 8.0
RobotState::RobotState()
{
        position.setZero();
        position_tacfoam.setZero();
        orientation.setZero();
        eigen_orientation.setZero();
        JntPosition.setZero(7);
        JntPosition_act.setZero(7);
//        JntPosition_mea.setZero(7);
        ctposition.setZero();
        contact_position.setZero();
        contact_nv.setZero();
        contactflag = false;
        gq = 0;
        eef_position_lastT.setZero();
        eef_orientation_lastT.setZero();
        eef_vel_g.setZero();
        eef_vel_g_lastT.setZero();
        eef_acc_g.setZero();
        eef_acc_g_lastT.setZero();
        theta_old = 0.0;
    }
RobotState::RobotState(Robot *r){
    position.setZero();
    position_tacfoam.setZero();
    orientation.setZero();
    eigen_orientation.setZero();
    JntPosition.setZero(7);
    JntPosition_act.setZero(7);
//    JntPosition_mea.setZero(7);
    ctposition.setZero();
    contact_position.setZero();
    contact_nv.setZero();
    contactflag = false;
    gq = 0;
    eef_position_lastT.setZero();
    eef_orientation_lastT.setZero();
    eef_vel_g.setZero();
    eef_vel_g_lastT.setZero();
    eef_acc_g.setZero();
    eef_acc_g_lastT.setZero();

    jntnum2name[0] = "base";
    jntnum2name[1] = "joint2_1";
    jntnum2name[2] = "joint2_2";
    jntnum2name[3] = "joint4_1";
    jntnum2name[4] = "joint4_2";
    jntnum2name[5] = "joint6_1";
    jntnum2name[6] = "joint6_2";
    jntnum2name[7] = "joint7";
    jntnum2name[8] = "eef";
    jntnum2name[9] = "robot_eef";

    theta_old = 0.0;
    old_hm.setIdentity(4,4);
    cur_hm.setIdentity(4,4);
    adj_matrix.setZero(6,6);
}

void RobotState::updated(Robot *r){
    for(int i = 0; i < 7; i++){
        r->get_joint_position_mea(JntPosition_mea);
        r->q(i) = JntPosition_mea[i];
    }
    if((r->get_robotname() == kuka_left)||((r->get_robotname() == kuka_right))){
        for(int i = 0; i <= 7; i++){
            r->worldToToolFkSolver->JntToCart(r->q,r->pose,i+4);
            robot_position[jntnum2name[i]] = kdl2eigen_position(r->pose);
            robot_orien[jntnum2name[i]] = kdl2eigen_orien(r->pose);
        }
    }
    if((r->get_robotname() == kuka_right)){
        r->worldToToolFkSolver->JntToCart(r->q,r->pose,11);
        robot_position["robot_eef"] = kdl2eigen_position(r->pose);
        robot_orien["robot_eef"] = kdl2eigen_orien(r->pose);
        if(r->toolname == sensing_pole)
            r->worldToToolFkSolver->JntToCart(r->q,r->pose,13);
        if(r->toolname == teensy_finger)
            r->worldToToolFkSolver->JntToCart(r->q,r->pose,12);
        robot_position["eef"] = kdl2eigen_position(r->pose);
        robot_orien["eef"] = kdl2eigen_orien(r->pose);
        gen_hm(robot_orien["eef"],robot_position["eef"],cur_hm);
        adj_matrix.topLeftCorner(3,3) = robot_orien["eef"];
        adj_matrix.bottomRightCorner(3,3) = robot_orien["eef"];
        adj_matrix.topRightCorner(3,3) = vectortoskew(robot_position["eef"]) * robot_orien["eef"];
//        robot_position["robot_eef"] = robot_position["eef"] = robot_position["joint7"];
//        robot_orien["robot_eef"] = robot_orien["eef"] = robot_orien["joint7"];
//        gen_hm(robot_orien["robot_eef"],robot_position["robot_eef"],cur_hm);
//        adj_matrix.topLeftCorner(3,3) = robot_orien["robot_eef"];
//        adj_matrix.bottomRightCorner(3,3) = robot_orien["robot_eef"];
//        adj_matrix.topRightCorner(3,3) = vectortoskew(robot_position["robot_eef"]) * robot_orien["robot_eef"];
    }
    if((r->get_robotname() == kuka_left)){
        r->worldToToolFkSolver->JntToCart(r->q,r->pose,11);
        robot_position["robot_eef"] = kdl2eigen_position(r->pose);
        robot_orien["robot_eef"] = kdl2eigen_orien(r->pose);
        if(r->toolname == sensing_pole)
            r->worldToToolFkSolver->JntToCart(r->q,r->pose,13);
        if(r->toolname == teensy_finger)
            r->worldToToolFkSolver->JntToCart(r->q,r->pose,12);
        robot_position["eef"] = kdl2eigen_position(r->pose);
        robot_orien["eef"] = kdl2eigen_orien(r->pose);
        gen_hm(robot_orien["eef"],robot_position["eef"],cur_hm);
        adj_matrix.topLeftCorner(3,3) = robot_orien["eef"];
        adj_matrix.bottomRightCorner(3,3) = robot_orien["eef"];
        adj_matrix.topRightCorner(3,3) = vectortoskew(robot_position["eef"]) * robot_orien["eef"];
    }
}
void RobotState::Est_eef_twist(Robot *r,Eigen::Vector3d& gv, Eigen::Vector3d& go){
    Eigen::MatrixXd I,devT;
    I.setIdentity(4,4);
    devT.setZero(4,4);
    devT = (I - old_hm * cur_hm.inverse())/r->gettimecycle();
    for(int i = 0; i < 3; i++)
        gv(i) = devT(i,3);
    go = skewtovector(devT.topLeftCorner(3,3));
//    std::cout<<"delta T"<<std::endl;
//    std::cout<<devT<<std::endl;
//    std::cout<<"lv "<<v(0)<<","<<v(1)<<","<<v(2)<<std::endl;
//    std::cout<<"lo "<<omega(0)<<","<<omega(1)<<","<<omega(2)<<std::endl;
}

void RobotState::twist_ltog(Eigen::Vector3d gv,Eigen::Vector3d go, Eigen::Vector3d& lv,Eigen::Vector3d& lo){
    Eigen::VectorXd gtw,ltw;
    ltw.setZero(6);
    gtw.setZero(6);
    gtw.head(3) = gv;
    gtw.tail(3) = go;
    ltw = adj_matrix.inverse()*gtw;
    lv = ltw.head(3);
    lo = ltw.tail(3);
}

Eigen::Vector3d RobotState::EstCtcPosition_KUKAPALM(Robot *r, myrmex_msg& tac_msg){
    Eigen::Vector3d p,l_p;
    Eigen::Vector3d p_kukaframe;
    p_kukaframe.setZero();
    l_p.setZero();
    p.setZero();
    //!-1 is used here because there is one assembling rotation 180 deg, 1 for left arm, 2 for left arm
    if(kuka_left == r->get_robotname()){
        l_p(0) = (-1) * (tac_msg.cogy - m_cog_cent)*0.005;
        l_p(1) = (-1) * (tac_msg.cogx - m_cog_cent)*0.005;
        l_p(2) = 0.029;
    }

    if(kuka_right == r->get_robotname()){
        l_p(0) = (tac_msg.cogy - m_cog_cent)*0.005;
        l_p(1) = (tac_msg.cogx - m_cog_cent)*0.005;
        l_p(2) = 0.029;
    }

    p = robot_position["eef"] + robot_orien["eef"] * l_p;
//    std::cout<<"contact p in right arm is: "<<p(0)<<","<<p(1)<<","<<p(2)<<std::endl;

    //for the transform matrix from global frame to right kuka base
    p_kukaframe = robot_orien["base"].transpose() * (p - robot_position["base"]);
    return p_kukaframe;
}
Eigen::Vector3d RobotState::EstCtcPosition_KUKAFINGER(){
    Eigen::Vector3d p;
    p.setZero();
    p = robot_orien["base"].transpose() * (robot_position["eef"] - robot_position["base"]);
    return p;
}


Eigen::Vector3d RobotState::EstCtcPosition_Ref(Robot *r, myrmex_msg& tac_msg){
    Eigen::Vector3d p,l_p;
    l_p.setZero();
    p.setZero();
    //!-1 is used here because there is one assembling rotation 180 deg, 1 for left arm, 2 for left arm
    if(kuka_left == r->get_robotname()){
        l_p(0) = (-1) * (tac_msg.cogy - m_cog_cent)*0.005;
        l_p(1) = (-1) * (tac_msg.cogx - m_cog_cent)*0.005;
        l_p(2) = 0.029;
    }

    if(kuka_right == r->get_robotname()){
        l_p(0) = (tac_msg.cogy - m_cog_cent)*0.005;
        l_p(1) = (tac_msg.cogx - m_cog_cent)*0.005;
        l_p(2) = 0.029;
    }
    p = robot_position["eef"] + robot_orien["eef"] * l_p;
    return p;

}

Eigen::Vector3d RobotState::EstCtcNormalVector_Ref(Robot *, myrmex_msg&){
    Eigen::Vector3d nv;
    nv.setZero();
    nv = robot_orien["eef"].col(3);
    return nv;
}

std::pair<Eigen::Vector3d,Eigen::Vector3d> RobotState::EstRobotEefRVel_InitF(Robot *r, bool& b){
    std::pair<Eigen::Vector3d,double> ax;
    std::pair<Eigen::Vector3d,Eigen::Vector3d> ax2;
    Eigen::Matrix3d R_init_cur;
    double theta_cur,rate_cur;
    R_init_cur.setZero();
    R_init_cur = robot_orien["eef"].transpose() * r->get_init_TM();
//    std::cout<<"R_init_cur "<<std::endl;
//    std::cout<<R_init_cur<<std::endl;
    ax = tm2axisangle_4(R_init_cur.transpose(),b);
    theta_cur = ax.second;
    ax2.first = ax.first;
    rate_cur = (theta_cur - theta_old)/r->gettimecycle();
    if(rate_cur > 0.1){
        rate_cur = rate_old;
    }
    if(fabs(rate_cur-rate_old) < 0.1){
        ax2.second(0) = rate_cur;
    }
    else{
        ax2.second(0) = rate_old;
    }
    ax2.second(1) = theta_cur;
    theta_old = theta_cur;
    rate_old = rate_cur;
    return ax2;
}

Eigen::Vector3d RobotState::EstRobotEefLVel_Ref(Robot *r){
    Eigen::Vector3d tmp_v;
    tmp_v.setZero();
    if((eef_position_lastT(0)==0)&&(eef_position_lastT(1)==0))
    {
        eef_vel_g.setZero();
        eef_position_lastT = robot_position["eef"];
        eef_orientation_lastT = robot_orien["eef"];
    }
    else
    {
        tmp_v = (robot_position["eef"] - eef_position_lastT)/r->gettimecycle();
        eef_vel_g = 0.5*(tmp_v + eef_vel_g_lastT);
        eef_position_lastT = robot_position["eef"];
        eef_orientation_lastT = robot_orien["eef"];
        eef_vel_g_lastT = eef_vel_g;
    }
    return eef_vel_g;
}

Eigen::Vector3d RobotState::EstRobotEefAcc_Ref(Robot *r){
//    eef_vel_g = (robot_position["eef"] - eef_position_lastT)/r->gettimecycle();
//    eef_acc_g = (eef_vel_g - eef_vel_g_lastT)/r->gettimecycle();
//    eef_vel_g_lastT = eef_acc_g;
//    eef_position_lastT = robot_position["eef"];
//    eef_orientation_lastT = robot_orien["eef"];
    return eef_acc_g;
}

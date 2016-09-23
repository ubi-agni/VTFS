#include "parametermanager.h"
#include <iostream>

ParameterManager::ParameterManager(){
    std::cout<<"empty constructor function"<<std::endl;

}

ParameterManager::ParameterManager(const std::string s = "left_arm_param.xml",TacSensorType t)
{
    std::cout<<"loading parameters"<<std::endl;
    map_name_dim[0] = "x";
    map_name_dim[1] = "y";
    map_name_dim[2] = "z";
    map_name_dim[3] = "roll";
    map_name_dim[4] = "pitch";
    map_name_dim[5] = "yaw";

    map_name_dim2[0] = "roll2";
    map_name_dim2[1] = "pitch2";
    map_name_dim2[2] = "yaw2";

    map_name_dim3[0] = ".e00";
    map_name_dim3[1] = ".e11";
    map_name_dim3[2] = ".e22";
    map_name_dim3[3] = ".e33";
    map_name_dim3[4] = ".e44";
    map_name_dim3[5] = ".e55";

    tac_map_task_name[CONTACT_POINT_TRACKING] = "ContactPointTracking";
    tac_map_task_name[CONTACT_FORCE_TRACKING] = "ContactForceTracking";
    tac_map_task_name[CONTACT_POINT_FORCE_TRACKING] = "ContactPointForceTracking";
    tac_map_task_name[SENSING_POLE_TRACKING] = "SensingPoleTracking";
    tac_map_task_name[Z_ORIEN_TRACKING] = "ZOrienTracking";
    tac_map_task_name[LINEAR_TRACKING] = "LineTracking";
    tac_map_task_name[COVER_OBJECT_SURFACE] = "CoverObjectSurface";
    tac_map_task_name[OBJECT_SURFACE_EXPLORING] = "ObjectSurfaceExploring";
    tac_map_task_name[LEARN_TACTOOL_CONTACT] = "LearnTactoolContact";
    tac_map_task_name[LEARN_TACTOOL_SLIDING] = "LearnTactoolSliding";
    tac_map_task_name[LEARN_TACTOOL_ROLLING] = "LearnTactoolRolling";

    pro_map_task_name[RLXP] = "Rlxp";
    pro_map_task_name[RLYP] = "Rlyp";
    pro_map_task_name[RLZP] = "Rlzp";
    pro_map_task_name[RRXP] = "Rrxp";
    pro_map_task_name[RRYP] = "Rryp";
    pro_map_task_name[RRZP] = "Rrzp";
    pro_map_task_name[RLXN] = "Rlxn";
    pro_map_task_name[RLYN] = "Rlyn";
    pro_map_task_name[RLZN] = "Rlzn";
    pro_map_task_name[RRXN] = "Rrxn";
    pro_map_task_name[RRYN] = "Rryn";
    pro_map_task_name[RRZN] = "Rrzn";
    pro_map_task_name[RP_NOCONTROL] = "Rno";
    pro_map_task_name[RP_LINEFOLLOW] = "Rlf";
    pro_map_task_name[RP_ROTATEFOLLOW] = "Rrf";
    pro_map_task_name[RP_BOTHFOLLOW] = "Rbf";

    force_map_task_name[F_MAINTAIN] = "ForceMaintain";
    force_map_task_name[F_CURVETRACKING] = "ForceCurveTracking";
    tst = t;
    std::cout<<"loading part finsihed"<<std::endl;

    loadCtrlParam(s);
}

void ParameterManager::load(TACTaskNameT tnt,ptree pt){
    tac_task_ctrl_param[tnt].kpp.setZero(6,6);
    tac_task_ctrl_param[tnt].kpi.setZero(6,6);
    tac_task_ctrl_param[tnt].kpd.setZero(6,6);
    tac_task_ctrl_param[tnt].kop.setZero(3,3);
    tac_task_ctrl_param[tnt].tsm.setZero(6,6);
    tac_task_ctrl_param[tnt].ttjkm.setZero(6,6);
    tac_task_ctrl_param[tnt].vsm.setZero(6,6);
    tac_task_ctrl_param[tnt].vtjkm.setZero(6,6);
    for(int i = 0; i < 6; i++){
        tac_task_ctrl_param[tnt].kpp(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+"."+map_name_dim[i]+".kp");
        tac_task_ctrl_param[tnt].kpi(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+"."+map_name_dim[i]+".ki");
        tac_task_ctrl_param[tnt].kpd(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+"."+map_name_dim[i]+".kd");
        tac_task_ctrl_param[tnt].tsm(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".tsm"+map_name_dim3[i]);
        tac_task_ctrl_param[tnt].vsm(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".vsm"+map_name_dim3[i]);
        tac_task_ctrl_param[tnt].vtjkm(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".vtjkm"+map_name_dim3[i]);
    }
    for(int i = 0; i < 3; i++){
        tac_task_ctrl_param[tnt].kop(i,i) =pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+"."+map_name_dim2[i]+".kp");
        tac_task_ctrl_param[tnt].ttjkm(i,i) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm"+map_name_dim3[i]);
    }
    //for myrmex sensor
    if(tst == Myrmex){
        tac_task_ctrl_param[tnt].ttjkm(3,1) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e33");
        tac_task_ctrl_param[tnt].ttjkm(4,0) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e44");
        tac_task_ctrl_param[tnt].ttjkm(5,5) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e55");
    }
    //for mid tactile sensor
    if(tst == MIDtip){
        //(3,2) map z deviation in fingertip frame to rotation
        tac_task_ctrl_param[tnt].ttjkm(3,2) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e33");
        //(4,1) map y deviation in fingertip frame to ratation
        tac_task_ctrl_param[tnt].ttjkm(4,1) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e44");
        //(5,0) map x deviation in fingertip frame to ratation
        tac_task_ctrl_param[tnt].ttjkm(5,0) = pt.get<double>("AdmittanceParams."+tac_map_task_name[tnt]+".ttjkm.e55");
    }
}

void ParameterManager::load(FORCETaskNameT fnt,ptree pt){
    force_task_ctrl_param[fnt].kpp.setZero(6,6);
    force_task_ctrl_param[fnt].kpi.setZero(6,6);
    force_task_ctrl_param[fnt].kpd.setZero(6,6);
    force_task_ctrl_param[fnt].kop.setZero(3,3);
    force_task_ctrl_param[fnt].tsm.setZero(6,6);
    force_task_ctrl_param[fnt].ttjkm.setZero(6,6);
    force_task_ctrl_param[fnt].vsm.setZero(6,6);
    force_task_ctrl_param[fnt].vtjkm.setZero(6,6);
    for(int i = 0; i < 6; i++){
        force_task_ctrl_param[fnt].kpp(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+"."+map_name_dim[i]+".kp");
        force_task_ctrl_param[fnt].kpi(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+"."+map_name_dim[i]+".ki");
        force_task_ctrl_param[fnt].kpd(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+"."+map_name_dim[i]+".kd");
        force_task_ctrl_param[fnt].tsm(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".tsm"+map_name_dim3[i]);
        force_task_ctrl_param[fnt].vsm(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".vsm"+map_name_dim3[i]);
        force_task_ctrl_param[fnt].vtjkm(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".vtjkm"+map_name_dim3[i]);
    }
    for(int i = 0; i < 3; i++){
        force_task_ctrl_param[fnt].kop(i,i) =pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+"."+map_name_dim2[i]+".kp");
        force_task_ctrl_param[fnt].ttjkm(i,i) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".ttjkm"+map_name_dim3[i]);
    }
    force_task_ctrl_param[fnt].ttjkm(3,1) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".ttjkm.e33");
    force_task_ctrl_param[fnt].ttjkm(4,0) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".ttjkm.e44");
    force_task_ctrl_param[fnt].ttjkm(5,5) = pt.get<double>("AdmittanceParams."+force_map_task_name[fnt]+".ttjkm.e55");
}


void ParameterManager::load(PROTaskNameT tnt,ptree pt){
    pro_task_ctrl_param[tnt].kpp.setZero(6,6);
    pro_task_ctrl_param[tnt].psm.setZero(6,6);
    for(int i = 0; i < 6; i++){
        pro_task_ctrl_param[tnt].kpp(i,i) = pt.get<double>("SelfCtrlParams."+pro_map_task_name[tnt]+".kp");
        pro_task_ctrl_param[tnt].psm(i,i) = pt.get<double>("SelfCtrlParams."+pro_map_task_name[tnt]+".psm"+map_name_dim3[i]);
    }
}
void ParameterManager::loadCtrlParam(std::string s){
    ptree pt;
    read_xml(s, pt);
    load(CONTACT_POINT_TRACKING,pt);
    load(CONTACT_FORCE_TRACKING,pt);
    load(CONTACT_POINT_FORCE_TRACKING,pt);
    load(SENSING_POLE_TRACKING,pt);
    load(Z_ORIEN_TRACKING,pt);
    load(LINEAR_TRACKING,pt);
    load(COVER_OBJECT_SURFACE,pt);
    load(OBJECT_SURFACE_EXPLORING,pt);
    load(RLXP,pt);
    load(RLYP,pt);
    load(RLZP,pt);
    load(RRXP,pt);
    load(RRYP,pt);
    load(RRZP,pt);
    load(RLXN,pt);
    load(RLYN,pt);
    load(RLZN,pt);
    load(RRXN,pt);
    load(RRYN,pt);
    load(RRZN,pt);
    load(RP_NOCONTROL,pt);
    load(RP_LINEFOLLOW,pt);
    load(RP_ROTATEFOLLOW,pt);
    load(RP_BOTHFOLLOW,pt);
    load(F_MAINTAIN,pt);
    load(F_CURVETRACKING,pt);
    //while myrmex is used for learning use the tool
    load(LEARN_TACTOOL_CONTACT,pt);
    load(LEARN_TACTOOL_SLIDING,pt);
    load(LEARN_TACTOOL_ROLLING,pt);

    stiff_ctrlpara.axis_stiffness[0] = pt.get<double>("StiffnessParams.stiffness.a1");
    stiff_ctrlpara.axis_stiffness[1] = pt.get<double>("StiffnessParams.stiffness.a2");
    stiff_ctrlpara.axis_stiffness[2] = pt.get<double>("StiffnessParams.stiffness.e1");
    stiff_ctrlpara.axis_stiffness[3] = pt.get<double>("StiffnessParams.stiffness.a3");
    stiff_ctrlpara.axis_stiffness[4] = pt.get<double>("StiffnessParams.stiffness.a4");
    stiff_ctrlpara.axis_stiffness[5] = pt.get<double>("StiffnessParams.stiffness.a5");
    stiff_ctrlpara.axis_stiffness[6] = pt.get<double>("StiffnessParams.stiffness.a6");

    stiff_ctrlpara.axis_damping[0] = pt.get<double>("StiffnessParams.damping.a1");
    stiff_ctrlpara.axis_damping[1] = pt.get<double>("StiffnessParams.damping.a2");
    stiff_ctrlpara.axis_damping[2] = pt.get<double>("StiffnessParams.damping.e1");
    stiff_ctrlpara.axis_damping[3] = pt.get<double>("StiffnessParams.damping.a3");
    stiff_ctrlpara.axis_damping[4] = pt.get<double>("StiffnessParams.damping.a4");
    stiff_ctrlpara.axis_damping[5] = pt.get<double>("StiffnessParams.damping.a5");
    stiff_ctrlpara.axis_damping[6] = pt.get<double>("StiffnessParams.damping.a6");
}
//void ActController::update_controller_para(){
//    loadCtrlParam();
//}

//void ActController::update_controller_para_stiffness(){
//    stiff_ctrlpara.axis_stiffness[0] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[1] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[2] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[3] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[4] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[5] = 1000+800;
//    stiff_ctrlpara.axis_stiffness[6] = 1000+800;

//    stiff_ctrlpara.axis_damping[0] = 0.2;
//    stiff_ctrlpara.axis_damping[1] = 0.2;
//    stiff_ctrlpara.axis_damping[2] = 0.2;
//    stiff_ctrlpara.axis_damping[3] = 0.2;
//    stiff_ctrlpara.axis_damping[4] = 0.2;
//    stiff_ctrlpara.axis_damping[5] = 0.2;
//    stiff_ctrlpara.axis_damping[6] = 0.2;
//}


/*
 ============================================================================
 Name        : StiffTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Stiffness and damping parameters setting test
 ============================================================================
 */

//for ICL
#include <ICLQt/Common.h>
#include <ICLCV/HoughLineDetector.h>
#include <ICLFilter/UnaryCompareOp.h>
#include <ICLCV/RegionDetector.h>
#include <ICLUtils/FPSLimiter.h>
#include <ICLGeom/Scene.h>
#include <ICLGeom/ComplexCoordinateFrameSceneObject.h>
#include <ICLMarkers/FiducialDetector.h>
#include <ICLUtils/Mutex.h>

#include "ComModule/ComOkc.h"
#include "RobotModule/KukaLwr.h"
#include "RobotModule/Robot.h"
#include "ControllerModule/proactcontroller.h"

//load the parameter which are stored in xml file
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "boost/foreach.hpp"
#include "ControllerModule/CtrlParam.h"

HSplit gui;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
ParameterManager *pm;

void init_ctrl_param(ParameterManager *ctrl){
    using boost::property_tree::ptree;
    ptree pt;
    std::string filename;
    filename = "left_arm_param.xml";
    read_xml(filename, pt);

    ctrl->stiff_ctrlpara.axis_stiffness[0] = pt.get<double>("StiffnessParams.stiffness.a1");
    ctrl->stiff_ctrlpara.axis_stiffness[1] = pt.get<double>("StiffnessParams.stiffness.a2");
    ctrl->stiff_ctrlpara.axis_stiffness[2] = pt.get<double>("StiffnessParams.stiffness.e1");
    ctrl->stiff_ctrlpara.axis_stiffness[3] = pt.get<double>("StiffnessParams.stiffness.a3");
    ctrl->stiff_ctrlpara.axis_stiffness[4] = pt.get<double>("StiffnessParams.stiffness.a4");
    ctrl->stiff_ctrlpara.axis_stiffness[5] = pt.get<double>("StiffnessParams.stiffness.a5");
    ctrl->stiff_ctrlpara.axis_stiffness[6] = pt.get<double>("StiffnessParams.stiffness.a6");

    ctrl->stiff_ctrlpara.axis_damping[0] = pt.get<double>("StiffnessParams.damping.a1");
    ctrl->stiff_ctrlpara.axis_damping[1] = pt.get<double>("StiffnessParams.damping.a2");
    ctrl->stiff_ctrlpara.axis_damping[2] = pt.get<double>("StiffnessParams.damping.e1");
    ctrl->stiff_ctrlpara.axis_damping[3] = pt.get<double>("StiffnessParams.damping.a3");
    ctrl->stiff_ctrlpara.axis_damping[4] = pt.get<double>("StiffnessParams.damping.a4");
    ctrl->stiff_ctrlpara.axis_damping[5] = pt.get<double>("StiffnessParams.damping.a5");
    ctrl->stiff_ctrlpara.axis_damping[6] = pt.get<double>("StiffnessParams.damping.a6");
}

void update_stiff_cb(){
}


void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
        kuka_left_arm->update_robot_stiffness();
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using controller to update the reference
        ac->update_robot_reference(kuka_left_arm);
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command();
        com_okc->controller_update = true;
    }
}


void init(){
    gui << (VBox()
        << Button("ustiff").handle("update_stiff_task")
        )
    << Show();
    gui["update_stiff_task"].registerCallback(utils::function(update_stiff_cb));
    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    ac = new ProActController(*pm);
    init_ctrl_param(pm);
    kuka_left_arm->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, ac->pm.stiff_ctrlpara.axis_damping);
}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run).exec();
}

/*
 ============================================================================
 Name        : CBFTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : CBF function test application
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


ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
ParameterManager* pm;


void update(){
}

void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
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
    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    std::cout<<"init part is finished"<<std::endl;
    ac = new ProActController(*pm);
}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run).exec();
}

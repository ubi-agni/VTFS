
/*
 ============================================================================
 Name        : NewComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : New communication component test
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


ComOkc *com_okc_left;
ComOkc *com_okc_right;
KukaLwr *left_arm;
KukaLwr *right_arm;
//boost::shared_ptr<ComOkc> comokc (new ComOkc(kuka_left,OKC_HOST,OKC_PORT));
//boost::shared_ptr<KukaLwr> left_arm (new KukaLwr(kuka_left,comokc));

void update(){
}

void run(){
    static FPSLimiter fpsl(250);
    fpsl.wait();
    std::cout<<"This is the new kuka control class test"<<std::endl;
}


void init(){
//    cbfleft.setSpeed(0.1);
//    while (!cbfleft.isConnected())
//      sleep(1);
//    //kuka autonomously locate itself at the predefined pose.
//    cbfleft.connect();
//    cbfleft.waitForFinished();
    com_okc_left = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc_right = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    while ((!com_okc_left->isConnected()) && (!com_okc_right->isConnected()))
      sleep(1);
    com_okc_left->connect();
    com_okc_left->waitForFinished();
    com_okc_right->connect();
    com_okc_right->waitForFinished();
    left_arm = new KukaLwr(kuka_left,*com_okc_left);
    right_arm = new KukaLwr(kuka_left,*com_okc_right);

    left_arm->waitForFinished();
    right_arm->waitForFinished();
    std::cout<<"init part is finished"<<std::endl;

}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run).exec();
}

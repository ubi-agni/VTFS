/*
 ============================================================================
 Name        : VisMarkerComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is communication test programe to get the raw data from visual fidutial marker from perception node
 ============================================================================
 */

/*
 *
*/


#include <iomanip>
#include "Timer.h"
#include "comrsb.h"
#include "msgcontenttype.h"

using namespace rsb;
ComRSB *com_rsb;
RsbDataType fid_feature;
markered_object_msg object_pose;
bool StopFlag;


void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void init(){
    com_rsb = new ComRSB();
    fid_feature = FiducialMarkerFeature;
    com_rsb->add_msg(fid_feature);
}

void run(){
    com_rsb->fiducialmarker_receive(object_pose);
    std::cout<<"vision output position: "<<object_pose.p<<std::endl;
    std::cout<<"vision output orien: "<<object_pose.roll<<","\
           <<object_pose.pitch<<","\
        <<object_pose.yaw<<std::endl;
}


int main(int argc, char* argv[])
{

    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    StopFlag = false;
    init();
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(30));
    thrd_getMsg.start(true);

    //register cb function
    com_rsb->register_external("/foo/closeprog",button_closeprog);
    //main thread is hanging
    while(!StopFlag){
    }

    //stop the timer
    thrd_getMsg.stop();
    return 0;
}




/*
 ============================================================================
 Name        : TipComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is communication test programe to get the raw data from teensy fingertip
 ============================================================================
 */

/*
 *
*/

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <sr_robot_msgs/UBI0All.h>
//#include <agni_utils/tactile_calibration.hpp>
#endif
#include <iomanip>
#include "UtilModule/Timer.h"
#include "ComModule/comrsb.h"

#ifdef HAVE_ROS
// ROS objects
ros::NodeHandle *nh;
#endif

//for rsb(currently icl remote gui communicate with kuka control via rsb)
#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>


using namespace rsb;
ComRSB *com_rsb;
bool StopFlag;


#ifdef HAVE_ROS
// receive Tactile data from UBI Fingertips
void
recvTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg)
{
    double val=0;
    // get access at all
    //boost::upgrade_lock<boost::shared_mutex> lock(update_mutex);
    // upgrade access to unique write access
    //boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

    // for first sensor each taxel
    std::cout <<"0"<<"\t"<<"1"<<"\t"<<"2"<<"\t"<<"3"<<"\t"<<"4"<<"\t"<<"5"<<"\t"<<"6"<<"\t"<<"7"<<"\t"<<"8"<<"\t"<<"9"<<"\t"<<"10"<<"\t"<<"11"<<"\t"<<std::endl;
    std::cout << std::fixed;
    for(size_t j = 0; j < msg->tactiles[0].distal.size(); ++j) {
        val= 1.0-(msg->tactiles[0].distal[j]/1023.0); //agni::tactileCalibration(msg->tactiles[0].distal[j], 1024, agni::UBI0);
        std::cout<<std::setprecision(5)<<val<<"\t";
    }
    std::cout<<std::endl;
}

#endif

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void run(){
#ifdef HAVE_ROS
    ros::Rate r(100);
    while (ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
#endif
}


int main(int argc, char* argv[])
{

    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    StopFlag = false;
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber	tacTipSub;
        if (argc > 1){
            std::cout<<"set hand by user"<<std::endl;
            std::string handname(argv[1]);
            tacTipSub=nh->subscribe("/"+handname+"/teensy/tactile", 1, // buffer size
                                                    &recvTipTactile);
          }
        else{
            //default is left hand
            std::cout<<"use the left hand as default"<<std::endl;
            tacTipSub=nh->subscribe("/lh/teensy/tactile", 1, // buffer size
                                                    &recvTipTactile);
        }
    #endif
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(20));
    thrd_getMsg.start(true);
    //start rsb
    com_rsb = new ComRSB();
    //register cb function
    com_rsb->register_external("/foo/closeprog",button_closeprog);
    //main thread is hanging
    while(!StopFlag){
    }

    //stop ros
    #ifdef HAVE_ROS
    ros::shutdown();
    delete nh;
    #endif

    //stop the timer
    thrd_getMsg.stop();
    return 1;
}




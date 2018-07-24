/*
 ============================================================================
 Name        : FTComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is communication test programe to get the force/torque from FT sensor
 ============================================================================
 */

/*
 *
*/

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
//#include <agni_utils/tactile_calibration.hpp>
#endif
#include <iomanip>
#include "Timer.h"
#include "comrsb.h"
#include <mutex>
#include "gamaft.h"
#include <geometry_msgs/WrenchStamped.h>

#ifdef HAVE_ROS
// ROS objects
ros::NodeHandle *nh;
#endif


using namespace rsb;
ComRSB *com_rsb;
bool StopFlag;

std::mutex mutex_ft;
//ptr for FT sensor on the left arm
gamaFT *ft_gama;

int counter;
#ifdef HAVE_ROS
// receive FT Sensor data
void
recvFT(const geometry_msgs::WrenchStampedConstPtr& msg){
//    std::cout <<"Fx"<<"\t"<<"Fy"<<"\t"<<"Fz"<<"\t"<<"Tx"<<"\t"<<"Ty"<<"\t"<<"Tz"<<std::endl;
    if (counter >50){
   std::cout << msg->wrench.force.x<<","<<msg->wrench.force.y<<","<<msg->wrench.force.z<<","<<msg->wrench.torque.x<<","<<msg->wrench.torque.y<<","<<msg->wrench.torque.z<<std::endl;
        counter = 0;
    }
   else{
       counter ++;
}
     ft_gama->raw_ft_f(0) = msg->wrench.force.x;
     ft_gama->raw_ft_f(1) = msg->wrench.force.y-0.1;
     ft_gama->raw_ft_f(2) = msg->wrench.force.z;
     ft_gama->raw_ft_t(0) = msg->wrench.torque.x;
     ft_gama->raw_ft_t(1) = msg->wrench.torque.y;
     ft_gama->raw_ft_t(2) = msg->wrench.torque.z;
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
    ft_gama = new gamaFT;
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber GammaFtSub;
        std::cout<<"Connect to ATI FT sensor"<<std::endl;
        GammaFtSub=nh->subscribe("/ft_sensor/wrench", 1, // buffer size 
                                &recvFT);
    #endif
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(20));
    thrd_getMsg.start(true);
//     //start rsb
//     com_rsb = new ComRSB();
//     //register cb function
//     com_rsb->register_external("/foo/closeprog",button_closeprog);
//     //main thread is hanging
    counter = 0;
    while(!StopFlag){
    }

    //stop ros
    #ifdef HAVE_ROS
    ros::shutdown();
    delete nh;
    #endif

    //stop the timer
    thrd_getMsg.stop();
    return 0;
}




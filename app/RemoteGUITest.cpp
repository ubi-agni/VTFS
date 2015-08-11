/*
 ============================================================================
 Name        : RemoteGUITest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : test remote gui application
 ============================================================================
 */
#include <stdlib.h>

#include <iostream>



//for rsb(currently icl remote gui communicate with kuka control via rsb)
#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>

#include "UtilModule/Timer.h"
#include "ComModule/comrsb.h"
#include "UtilModule/msgcontenttype.h"

using namespace rsb;
ComRSB *com_rsb;
bool StopFlag;

void function_slider(boost::shared_ptr<std::string> data) {
    std::cout<<"slider value "<<*data<<std::endl;
}

void function_button(boost::shared_ptr<std::string> data) {
    std::cout<<"click botton "<<*data<<std::endl;
}

void stop_button_cb(boost::shared_ptr<std::string> data) {
    std::cout<<"click stop botton "<<*data<<std::endl;
    StopFlag = true;

}

void function_checkb(boost::shared_ptr<std::string> data) {
    std::cout<<"check box is "<<*data<<std::endl;
}

void run(){

}

int main(int argc, char** argv) {
    StopFlag = false;

    boost::function<void(boost::shared_ptr<std::string>)> fun_slider(function_slider);
    boost::function<void(boost::shared_ptr<std::string>)> fun_button(function_button);
    boost::function<void(boost::shared_ptr<std::string>)> stop_button(stop_button_cb);
    boost::function<void(boost::shared_ptr<std::string>)> test_checkb(function_checkb);

    com_rsb = new ComRSB();
    com_rsb->register_external("/foo/impbutton",fun_button);
    com_rsb->register_external("/foo/stopbutton",stop_button);
    com_rsb->register_external("/foo/slider",fun_slider);
    com_rsb->register_external("/foo/theCheckBox",test_checkb);
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1000));
    thrd_getMsg.start(true);

    while(!StopFlag){
    }
    std::cout<<"stop is clicked!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    thrd_getMsg.stop();
    return 1;
}
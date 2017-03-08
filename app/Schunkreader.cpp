/*
 ============================================================================
 Name        : Schunk joint angle reader.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : routing for testing schunk js reading via rsb
 ============================================================================
 */
#include "Timer.h"
#include "comrsb.h"

ComRSB *com_rsb;
RsbDataType rdtschunkjs;
//using mutex locking controller ptr while it is switching.
std::mutex mutex_schunkjs2;

//get schunk joint angle
std::vector<double> schunkjs;

void schunkJSviarsb(){
    mutex_schunkjs2.lock();
    com_rsb->schunkjs_receive(schunkjs);
    std::cout<<"schunk joint angle are ";
    for(int i = 0; i < schunkjs.size(); i++){
        std::cout<<schunkjs.at(i)<<",";
    }
    std::cout<<std::endl;
    mutex_schunkjs2.unlock();
}

int main(int argc, char* argv[]){
    //init Rsb
    com_rsb = new ComRSB();
    rdtschunkjs = SchunkJS;
    com_rsb->add_msg(rdtschunkjs);
    //start Schunk hand read thread
    Timer thrd_schunk_read(schunkJSviarsb);
    thrd_schunk_read.setSingleShot(false);
    thrd_schunk_read.setInterval(Timer::Interval(100));
    thrd_schunk_read.start(true);
    while(1){
//         std::cout<<"in the lopp"<<std::endl;
//         mutex_schunkjs.lock();
//     com_rsb->schunkjs_receive(schunkjs);
//     std::cout<<"schunk joint angle are ";
//     for(int i = 0; i < schunkjs.size(); i++){
//         std::cout<<schunkjs.at(i)<<",";
//     }
//     std::cout<<std::endl;
//     mutex_schunkjs.unlock();
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
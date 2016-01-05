/*
 ============================================================================
 Name        : MyrmexComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : test the communication of myrmex via rsb
             :start perception node on augustus and send tactile, this application can
              whether the myrmex communication is good
 ============================================================================
 */


#include <iomanip>
#include <iostream>

#include "Timer.h"
#include "comrsb.h"
//#include "Util.h"

//#include "RebaType.h"
#include "msgcontenttype.h"


ComRSB *com_rsb;
RsbDataType rdtleftkuka;
RsbDataType rdtlefttac;
myrmex_msg left_myrmex_msg;
kuka_msg left_kuka_msg;

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    com_rsb->tactile_receive(left_myrmex_msg,"leftmyrmex");
    std::cout<<"tactile output"<<std::endl;
    std::cout<<"myrmex readout "<<left_myrmex_msg.cogx<<","<<left_myrmex_msg.cogy<<std::endl;
}

void init(){
    com_rsb = new ComRSB();
    rdtleftkuka = LeftKukaEff;
    rdtlefttac = LeftMyrmex;
    com_rsb->add_msg(rdtleftkuka);
    com_rsb->add_msg(rdtlefttac);
}


int main(int argc, char* argv[])
{
    init();
    //start myrmex read thread
    Timer thrd_myrmex_read(tactileviarsb);
    thrd_myrmex_read.setSingleShot(false);
    thrd_myrmex_read.setInterval(Timer::Interval(5));
    thrd_myrmex_read.start(true);
    //main thread is hanging
    while(1){
    }
    thrd_myrmex_read.stop();
    return 0;
}






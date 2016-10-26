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
#include <string>

#include "Timer.h"
#include "comrsb.h"
//#include "Util.h"

//#include "RebaType.h"
#include "msgcontenttype.h"


ComRSB *com_rsb;
RsbDataType rdtrightkuka;
RsbDataType rdtrighttac;
RsbDataType rdtleftkuka;
RsbDataType rdtlefttac;
myrmex_msg m_myrmex_msg;
kuka_msg kuka_msg;
std::string myrmex_name;

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    if(myrmex_name == "rmyrmex"){
    com_rsb->tactile_receive(m_myrmex_msg,"rightmyrmex");
    std::cout<<"tactile output"<<std::endl;
    std::cout<<"right myrmex readout "<<m_myrmex_msg.cogx<<","<<m_myrmex_msg.cogy<<std::endl;
    }
    else{
        com_rsb->tactile_receive(m_myrmex_msg,"leftmyrmex");
        std::cout<<"tactile output"<<std::endl;
        std::cout<<"left myrmex readout "<<m_myrmex_msg.cogx<<","<<m_myrmex_msg.cogy<<std::endl;
    }
}

void init(){
    com_rsb = new ComRSB();
    if(myrmex_name == "rmyrmex"){
        rdtrightkuka = RightKukaEff;
        rdtrighttac = RightMyrmex;
        com_rsb->add_msg(rdtrightkuka);
        com_rsb->add_msg(rdtrighttac);
    }
    else{
        rdtleftkuka = LeftKukaEff;
        rdtlefttac = LeftMyrmex;
        com_rsb->add_msg(rdtleftkuka);
        com_rsb->add_msg(rdtlefttac);
    }
}


int main(int argc, char* argv[])
{
    if (argc > 1) {
        myrmex_name = argv[1];
        init();
        std::cout<<"start"<<std::endl;
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
    else{
        std::cout<<"please specify which myrmex is used"<<std::endl;
        return 1;
    }
}






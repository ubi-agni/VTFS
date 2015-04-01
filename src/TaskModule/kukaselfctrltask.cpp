#include "kukaselfctrltask.h"

void KukaSelfCtrlTask::switchtotask(PROTaskNameT tn){
    curtaskname.prot = tn;
}

void KukaSelfCtrlTask::switchtoglobalframe(){
    mft = GLOBAL;
}
void KukaSelfCtrlTask::switchtolocalframe(){
    mft = LOCAL;
}

KukaSelfCtrlTask::KukaSelfCtrlTask(PROTaskNameT tn)
{
    curtaskname.prot = tn;
}



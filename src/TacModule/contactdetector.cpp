#include "contactdetector.h"


ContactDetector::ContactDetector()
{
    deque_len = 5;
    tac_ctc_deque.assign(deque_len, false);
    ctcs = nocontact;
}


ctc_status ContactDetector::get_ctc_status(bool b){
    tac_ctc_deque.push_back(b);
    tac_ctc_deque.pop_front();
    if((tac_ctc_deque.at(0) == true)&&(tac_ctc_deque.at(1) == true)&&\
            (tac_ctc_deque.at(2) == true)&&(tac_ctc_deque.at(3) == true)&&\
            (tac_ctc_deque.at(4) == true)){
        //if all true, then stable contact
        return stablecontact;
    }
    if((tac_ctc_deque.at(0) == true)&&(tac_ctc_deque.at(1) == true)&&\
            (tac_ctc_deque.at(2) == true)&&(tac_ctc_deque.at(3) == false)&&\
            (tac_ctc_deque.at(4) == false)){
        return leavingcontact;
    }
    if((tac_ctc_deque.at(0) == false)&&(tac_ctc_deque.at(1) == false)&&\
            (tac_ctc_deque.at(2) == false)&&(tac_ctc_deque.at(3) == false)&&\
            (tac_ctc_deque.at(4) == false)){
        return nocontact;
    }
    if((tac_ctc_deque.at(0) == false)&&(tac_ctc_deque.at(1) == false)&&\
            (tac_ctc_deque.at(2) == true)&&(tac_ctc_deque.at(3) == true)&&\
            (tac_ctc_deque.at(4) == true)){
        return initcontact;
    }
    else{
        return uncertaincontact;
    }
}

bool ContactDetector::isContactArea(double x,double y){
    if((x>=2)&&(x<=13)&&(y>=2)&&(y<=13)){
        return true;
    }
    else{
        return false;
    }

}

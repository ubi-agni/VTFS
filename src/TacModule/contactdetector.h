#ifndef CONTACTDETECTOR_H
#define CONTACTDETECTOR_H
#include <deque>

enum ctc_status{
    nocontact = 0,
    initcontact = 1,
    stablecontact = 2,
    leavingcontact = 3,
    uncertaincontact = 4
};

class ContactDetector{
public:
    ContactDetector();
    //use a deque to assign the contact status, current we are using five element v
    //if all element are true, stable contact
    //if first three are true, others are false: leaving contact
    //if all five are false: no contact
    //if first two are false, others are true: init contact
    std::deque<bool> tac_ctc_deque;
    int deque_len;
    ctc_status ctcs;
    ctc_status get_ctc_status(bool b);
    bool isContactArea(double x,double y);
    bool isSlideOK(double x, double y, double dirx, double diry);
};
#endif // CONTACTDETECTOR_H

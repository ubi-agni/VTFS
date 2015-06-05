#include <iostream>

#include "UtilModule/Timer.h"

using namespace std;

void printout(){
    std::cout << "Hello!" << endl;
}

int main(void)
{
    Timer tHello(printout);

    tHello.setSingleShot(false);
    tHello.setInterval(Timer::Interval(1000));
    tHello.start(true);

    Timer tStop([&]()
    {
        tHello.stop();
    });

    tStop.setSingleShot(true);
    tStop.setInterval(Timer::Interval(5000));
    tStop.start();

    return 0;
}

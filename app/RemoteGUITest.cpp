#include <stdlib.h>

#include <iostream>

#include <rsc/misc/SignalWaiter.h>

#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>

#include "UtilModule/Timer.h"
#include "ComModule/comrsb.h"
#include "UtilModule/msgcontenttype.h"

using namespace rsb;
ComRSB *com_rsb;
RsbDataType gv;
gui_msg g_msg;

void function(boost::shared_ptr<std::string> data) {

    std::cout<<"data get from gui2222 "<<*data<<std::endl;
//    //check whether button is clicked
//    gm.bt_click = data->bt_click();
//    //check the value of the slider
//    gm.sl = data->sl();
//    //check whether the checkbox is checked
//    gm.cb = data->cb();

}

void printMsg(){
    com_rsb->gui_receive(g_msg);
    std::cout<<g_msg.bt_click<<","<<g_msg.sl<<","<<g_msg.cb<<std::endl;
    std::cout << "Received event: " << std::endl;
}

int main(int argc, char** argv) {

//    rsc::misc::initSignalWaiter();

//    // First get a factory instance that is used to create RSB
//    // objects.
//    Factory& factory = getFactory();

//    // Set up the scope to receive on either from the command line
//    // argument or use the default scope of the informer example.
//    Scope scope((argc > 1) ? argv[1] : "/example/informer");

//    // Create a listener that asynchronously receives events from the
//    // bus and dispatches them to registered handlers.
//    ListenerPtr listener = factory.createListener(scope);

//    // Add a handler that is notified about every new event.  This
//    // time a special handler instance is used that wraps a function
//    // pointer of a function that is only interested in the received
//    // data contained in the event and not the additional meta data
//    // provided by the event instance. Other handlers exist that also
//    // receive Event instances, either as class instances or by
//    // wrapping function pointers.
//    listener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&printData)));

//    // As events are received asynchronously we have to wait here for
//    // them.
//    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
    rsc::misc::initSignalWaiter();

    boost::function<void(boost::shared_ptr<std::string>)> fun(function);

    com_rsb = new ComRSB();
    gv = GuiEvent;
    com_rsb->add_msg(gv);
    com_rsb->register_external(gv, fun);
    Timer thrd_getMsg(printMsg);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1000));
    thrd_getMsg.start(true);

    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

}

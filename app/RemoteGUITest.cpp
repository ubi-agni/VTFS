#include <stdlib.h>

#include <iostream>

#include <rsc/misc/SignalWaiter.h>

#include <rsb/Handler.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>

using namespace rsb;

void printData(boost::shared_ptr<std::string> e) {
    std::cout << "Received event: " << *e << std::endl;
}

int main(int argc, char** argv) {

    rsc::misc::initSignalWaiter();

    // First get a factory instance that is used to create RSB
    // objects.
    Factory& factory = getFactory();

    // Set up the scope to receive on either from the command line
    // argument or use the default scope of the informer example.
    Scope scope((argc > 1) ? argv[1] : "/example/informer");

    // Create a listener that asynchronously receives events from the
    // bus and dispatches them to registered handlers.
    ListenerPtr listener = factory.createListener(scope);

    // Add a handler that is notified about every new event.  This
    // time a special handler instance is used that wraps a function
    // pointer of a function that is only interested in the received
    // data contained in the event and not the additional meta data
    // provided by the event instance. Other handlers exist that also
    // receive Event instances, either as class instances or by
    // wrapping function pointers.
    listener->addHandler(HandlerPtr(new DataFunctionHandler<std::string> (&printData)));

    // As events are received asynchronously we have to wait here for
    // them.
    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

}

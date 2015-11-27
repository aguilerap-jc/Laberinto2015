#include "tactile.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include <althread/alcriticalsection.h>

#include "laberinto.h"

laberinto nuevo = laberinto();
bool ejecutando=false;

Tactile::Tactile(boost::shared_ptr<AL::ALBroker> broker, const std::string& name): AL::ALModule(broker, name), fCallbackMutex(AL::ALMutex::createALMutex()) {
    setModuleDescription("This module presents how to subscribe to a simple event (here tactile) and use a callback method.");

    functionName("onFrontTactileTouched", getName(), "Method called when the Front Tactile is pressed.");
    BIND_METHOD(Tactile::onFrontTactileTouched)

    functionName("onRearTactileTouched", getName(), "Method called when the Back Tactile is pressed.");
    BIND_METHOD(Tactile::onRearTactileTouched)
}

Tactile::~Tactile(){
    fMemoryProxy.unsubscribeToEvent("onFrontTactileTouched", "Tactile");
    fMemoryProxy2.unsubscribeToEvent("onRearTactileTouched", "Tactile");
}

void Tactile::init() {
    try {
        /** Create a proxy to ALMemory.
    */
        fMemoryProxy = AL::ALMemoryProxy(getParentBroker());

        fState = fMemoryProxy.getData("FrontTactilTouched");
        /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
        fMemoryProxy.subscribeToEvent("FrontTactilTouched", "Tactile",
                                      "onFrontTactileTouched");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }

    try {
        /** Create a proxy to ALMemory.
    */
        fMemoryProxy2 = AL::ALMemoryProxy(getParentBroker());

        fState2 = fMemoryProxy2.getData("RearTactilTouched");
        /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
        fMemoryProxy2.subscribeToEvent("RearTactilTouched", "Tactile",
                                       "onRearTactileTouched");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }

}

void Tactile::onFrontTactileTouched() {
    qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;

    fState =  fMemoryProxy.getData("FrontTactilTouched");
    if ((fState  > 0.5f)&&(ejecutando==false)) {
        ejecutando=true;
        nuevo.setEjecutar(true);
        nuevo.resolver();
        return;
    }
    try {
        //fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
        //fTtsProxy.say("Front tactile pressed");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }
}

void Tactile::onRearTactileTouched() {
    qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;

    fState2 =  fMemoryProxy2.getData("RearTactilTouched");
    if ((fState2  > 0.5f)) {
        std::cout<<"Detener resolución"<<std::endl;
        ejecutando = false;
        nuevo.setEjecutar(false);
        return;
    }
    try {
        //fTtsProxy2 = AL::ALTextToSpeechProxy(getParentBroker());
        //fTtsProxy2.say("Rear tactile pressed");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }
}

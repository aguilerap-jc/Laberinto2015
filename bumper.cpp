/**
 * Copyright (c) 2011 Aldebaran Robotics
 */

#include "bumper.h"

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include <althread/alcriticalsection.h>

#include <alproxies/almotionproxy.h>


Bumper::Bumper(
        boost::shared_ptr<AL::ALBroker> broker,
        const std::string& name): AL::ALModule(broker, name),
    fCallbackMutex(AL::ALMutex::createALMutex())
{
    setModuleDescription("This module presents how to subscribe to a simple event (here RightBumperPressed) and use a callback method.");

    functionName("onRightBumperPressed", getName(), "Method called when the right bumper is pressed.");
    BIND_METHOD(Bumper::onRightBumperPressed)

    functionName("onLeftBumperPressed", getName(), "Method called when the left bumper is pressed.");
    BIND_METHOD(Bumper::onLeftBumperPressed)
}

Bumper::~Bumper() {
    fMemoryProxyR.unsubscribeToEvent("onRightBumperPressed", "Bumper");
    fMemoryProxyL.unsubscribeToEvent("onLeftBumperPressed", "Bumper");
}

void Bumper::init() {
    try {
        /** Create a proxy to ALMemory.
    */
        fMemoryProxyR = AL::ALMemoryProxy(getParentBroker());

        fStateR = fMemoryProxyR.getData("RightBumperPressed");
        /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
        fMemoryProxyR.subscribeToEvent("RightBumperPressed", "Bumper",
                                      "onRightBumperPressed");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }

    try {
        /** Create a proxy to ALMemory.
    */
        fMemoryProxyL = AL::ALMemoryProxy(getParentBroker());

        fStateL= fMemoryProxyL.getData("RightBumperPressed");
        /** Subscribe to event LeftBumperPressed
    * Arguments:
    * - name of the event
    * - name of the module to be called for the callback
    * - name of the bound method to be called on event
    */
        fMemoryProxyL.subscribeToEvent("LeftBumperPressed", "Bumper",
                                      "onLeftBumperPressed");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }
}

void Bumper::onRightBumperPressed() {
    qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;
    /**
  * As long as this is defined, the code is thread-safe.
  */
    AL::ALCriticalSection sectionR(fCallbackMutex);

    /**
  * Check that the bumper is pressed.
  */
    fStateR =  fMemoryProxyR.getData("RightBumperPressed");
    if (fStateR  > 0.5f) {
        //AL::ALMotionProxy movimiento;
        //movimiento.moveTo(-0.2,0,0);
        return;
    }
    try {
        //fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
        //fTtsProxy.say("Right bumper pressed");

    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }
}

void Bumper::onLeftBumperPressed() {
    qiLogInfo("module.example") << "Executing callback method on right bumper event" << std::endl;
    /**
  * As long as this is defined, the code is thread-safe.
  */

    /**
  * Check that the bumper is pressed.
  */
    fStateL =  fMemoryProxyL.getData("LeftBumperPressed");
    if (fStateL  > 0.5f) {
       // AL::ALMotionProxy movimiento;
        //movimiento.moveTo(-0.2,0,0);
        return;
    }
    try {
        //fTtsProxy = AL::ALTextToSpeechProxy(getParentBroker());
        //fTtsProxy.say("Left bumper pressed");
    }
    catch (const AL::ALError& e) {
        qiLogError("module.example") << e.what() << std::endl;
    }
}

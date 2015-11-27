#ifndef TACTILE_H
#define TACTILE_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <string>

#include <alproxies/almemoryproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <althread/almutex.h>

namespace AL
{
  class ALBroker;
}

class Tactile : public AL::ALModule
{
  public:
    Tactile(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);

    virtual ~Tactile();

    /** Overloading ALModule::init().
    * This is called right after the module has been loaded
    */
    virtual void init();

    /**
    * This method will be called every time the event RightBumperPressed is raised.
    */
    void onFrontTactileTouched();
    void onRearTactileTouched();

  private:
    AL::ALMemoryProxy fMemoryProxy;
    AL::ALMemoryProxy fMemoryProxy2;
    AL::ALTextToSpeechProxy fTtsProxy;
    AL::ALTextToSpeechProxy fTtsProxy2;

    boost::shared_ptr<AL::ALMutex> fCallbackMutex;

    float fState;
    float fState2;
};

#endif // TACTILE_H

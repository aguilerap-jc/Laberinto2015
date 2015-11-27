/**
 * @author Emilie Wirbel
 *
 * \section Description
 * This file was generated by Aldebaran Robotics ModuleGenerator
 */

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "tactile.h"
#include "bumper.h"

#ifdef BUMPER_IS_REMOTE
# define ALCALL
#else
# ifdef _WIN32
#  define ALCALL __declspec(dllexport)
# else
#  define ALCALL
# endif
#endif

extern "C"
{
  ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
    AL::ALModule::createModule<Tactile>( pBroker, "Tactile" );
    AL::ALModule::createModule<Bumper>( pBroker, "Bumper");

    return 0;
  }

  ALCALL int _closeModule()
  {
    return 0;
  }
}

#ifdef BUMPER_IS_REMOTE
  int main(int argc, char *argv[])
  {
    // pointer to createModule
    TMainType sig;
    sig = &_createModule;
    // call main
    ALTools::mainFunction("Tactile", argc, argv, sig);
    ALTools::mainFunction("Bumper", argc, argv, sig);
  }
#endif

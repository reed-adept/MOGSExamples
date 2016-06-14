/*

Copyright (c) 2014-2015 Adept Technology Inc.
All rights reserved.  

Redistribution of this example source code, with or without modification, is 
permitted provided that the following conditions are met:   
-    Redistributions must retain the above copyright notice, 
     this list of conditions and the following disclaimer.  
-    Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Adept Technology, Inc.

Adept Technology, Inc. makes no warranty as to the suitability of this material
for use by the recipient, and assumes no responsibility for any consequences
resulting from such use. 

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/

#ifndef REGULARSTOPACTION_HH
#define REGULARSTOPACTION_HH

#include "Aria.h"
#include "ArNetworking.h"



/** This action regularly stops the robot after it has gone a certain distance.
 * A calback is called.  This action continues to keep the robot stopped until
 * resume() is called; it also immediately resumes if the callback returns
 * true. The callback must return as soon as possible. It must not block for 
 * any indeterminate amount of time, and it must return within 100ms or less,
 * since it is called from the ArRobot task cycle (as part of the ArActions
 * phase).  If you want to perform any significant computation, IO, or are
 * waiting for anything, this callback sholud create or signal another thread.
 * This thread can call resume() when done.   You may also need to increase
 * ARNL's path planning timeout (SecsToFail parameter) to be greater than your
 * expected stop time.
 */
class RegularStopAction : public virtual ArAction
{
  double stopdist;
  ArRetFunctor<bool> *callback; // todo make it a callbacklist
  bool stopped;
  bool stopping;
  bool enabled;
  ArPose lastpoint;
  ArFunctorC<RegularStopAction> myActivateCmdCB;
  ArFunctorC<RegularStopAction> myDeactivateCmdCB;
  ArRetFunctorC<bool, RegularStopAction> myProcessConfigCB;
  ArActionDesired desiredAction;
  ArMutex mutex; 
protected:
  void lock() { mutex.lock(); }
  void unlock() { mutex.unlock(); }
public:
  RegularStopAction(double default_stopdist, ArRetFunctor<bool> *_cb = NULL, const char *name = "RegularStopAction", ArServerHandlerCommands *cmds = NULL) :
    ArAction(name),
    stopdist(default_stopdist), // may be replaced by value from ArConfig later
    callback(_cb), 
    stopped(false),
    stopping(false),
    enabled(true),
    myActivateCmdCB(this, &RegularStopAction::activate),
    myDeactivateCmdCB(this, &RegularStopAction::deactivate),
    myProcessConfigCB(this, &RegularStopAction::processConfig),
    mutex(true) // true for recursive, needs to allow recursive locking
  {
    Aria::getConfig()->addParam(ArConfigArg("StopDistance", &stopdist, "During path following, stop and perform action after robot has gone this distance (mm)", 1), "Regular Stop Action");
    Aria::getConfig()->addParam(ArConfigArg("Enabled", &enabled, "Don't activate if false"), "Regular Stop Action");
    Aria::getConfig()->addProcessFileCB(&myProcessConfigCB, 30);
    if(cmds) {
      cmds->addCommand("RegularStopAction:activate", "Activate the regular stop action", &myActivateCmdCB);
      cmds->addCommand("RegularStopAction:deactivate", "Deactivate the regular stop action", &myDeactivateCmdCB);
    }
  }
  
  void addCallback(ArRetFunctor<bool> *cb)
  {
    // todo make it a callback list
    callback = cb;
  }

  virtual void activate() 
  {
    lock();
    if(!enabled) {
      ArLog::log(ArLog::Normal, "RegularStopAction: Disabled from config, not activating.");
      unlock();
      return;
    }
    unlock();
    ArLog::log(ArLog::Normal, "RegularStopAction: Activated");
    ArAction::activate();
    resume();
  }

  virtual void deactivate()
  {
    ArLog::log(ArLog::Normal, "RegularStopAction: Deactivated");
    ArAction::deactivate();
  }
    
  void resume() 
  {
    lock();
    ArLog::log(ArLog::Normal, "RegularStopAction: Allowing navigation to resume");
    lastpoint = getRobot()->getPose();
    desiredAction.reset();
    stopped = false;
    stopping = false;
    unlock();
  }
 
  ArActionDesired* fire(ArActionDesired currentDesired)
  {
    lock();
    desiredAction.reset();
    const ArPose currPose = getRobot()->getPose();
    const double dist = currPose.findDistanceTo(lastpoint);
    if(!stopped && dist >= stopdist )
    {
      // stopped yet?
      if(getRobot()->isStopped())
      {
        // finished decelerating and we're now stopped
        ArLog::log(ArLog::Normal, "%s: Stopped.", getName());
        stopped = true;
        bool cont = callback->invokeR();
        if(cont)
        {
          // callback returned true, resume
          resume();
          unlock();
          return &desiredAction;
        }
      }
      else
      {
        // should stop, set flag
        ArLog::log(ArLog::Normal, "%s: Stopping...", getName());
        stopping = true;
      }
    }

    if(stopping || stopped)
    {
      // remain stopped until resumed
      desiredAction.setVel(0);
      desiredAction.setRotVel(0);
    }

    unlock();
    return &desiredAction;
  }

  bool processConfig()
  {
    if(enabled) activate();
    else deactivate();
    return true;
  }

  void enable()
  {
    lock();
    enabled = true;
    processConfig();
    unlock();
  }
  
  void disable()
  {
    lock();
    enabled = false;
    processConfig();
    unlock();
  }
};

#endif

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

#ifndef EXAMPLEPAUSETASK_HH
#define EXAMPLEPAUSETASK_HH

#include "Aria.h"
#include "ArNetworking.h"
#include "RegularStopAction.h"

class ScopedLock { 
 private: 
   ArMutex& mtx; 
 public: 
   ScopedLock(ArMutex& m) : mtx(m) { 
     mtx.lock(); 
   } 
   ~ScopedLock() { 
     mtx.unlock(); 
   } 
   void lock() { mtx.lock(); } 
   void unlock() { mtx.unlock(); } 
   void sleepUnlocked(unsigned int ms) { unlock(); ArUtil::sleep(ms); lock(); }
 }; 

// Example target of RegularStopAction
// Note, an elaboration on ArASyncTask may be added to ARIA or ARNL in the
// future which takes care of most of the threading, mutexes, functors, state, 
// commands, some ArConfig stuff, etc. 
// to the task implementation can be smaller and cleaner.
class ExamplePauseTask : public virtual ArASyncTask
{
  ArRetFunctorC<bool, ExamplePauseTask> myStartCallback;
  ArFunctorC<ExamplePauseTask> myStopCallback;
  double myTime;
  ArRobot *myRobot;
  RegularStopAction *myAction;
  bool keepPausing;
  ArServerHandlerPopup *popupServer;
  ArServerHandlerPopupInfo popupInfo;
  ArFunctor2C<ExamplePauseTask, ArTypes::Byte4, int> handlePopupCB;
  ArTypes::Byte4 popupID;
  ArGPS *myGPS;
  ArMutex mutex;

public:
  ExamplePauseTask(double time, ArRobot *robot, ArGPS *gps, RegularStopAction *action, ArServerHandlerPopup *_popupServer = NULL) :
    myStartCallback(this, &ExamplePauseTask::start), 
    myStopCallback(this, &ExamplePauseTask::stop),
    myTime(time), myRobot(robot), myAction(action),
    keepPausing(false),
    popupServer(_popupServer),
    popupInfo(NULL, "Example Pause Task", "Pausing...", ArServerHandlerPopup::INFORMATION, 1, 0, -1, NULL, "Resume", "Resuming...", "Ignore", "Ignoring..."),
    handlePopupCB(this, &ExamplePauseTask::handlePopup),
    popupID(-1),
    myGPS(gps)
  {
    Aria::getConfig()->addParam(ArConfigArg("PauseTime", &myTime, "Time to pouse and do nothing before end of task (secs)", 0.0), "Pause Task");
    action->setCallback(&myStartCallback);
    action->setDeactivatedCallback(&myStopCallback);
  }

  ArRetFunctor<bool> * getStartCallback() { return &myStartCallback; }


protected:
  bool start() 
  {
    runAsync();
    return false;
  }

  void stop()
  {
    ScopedLock lock(mutex);
    if(popupServer && popupID != -1) popupServer->closePopup(popupID, "Interrupted");
    popupID = -1;
    stopRunning();
  }

  virtual const char *getThreadActivity() 
  {
    ScopedLock lock(mutex);
    if(!getRunning()) return "Not running";
    if(keepPausing) return "Paused";
    else return "Resuming";
  }

  void *runThread(void*) 
  {
    ScopedLock lock(mutex);
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: started, waiting %.0f ms...", myTime*1000);
    ArTime sleepTime;
    if(popupServer) popupID = popupServer->createPopup(&popupInfo, &handlePopupCB);
    keepPausing = true;
    while(getRunning() && keepPausing)
    {
      if(sleepTime.secSince() >= myTime)
        	break;
      lock.sleepUnlocked(200); //ArUtil::sleep(200);
    }
    if(!getRunning()) return 0;
    keepPausing = false;
    if(popupServer && popupID != -1) popupServer->closePopup(popupID, "Resuming");
    popupID = -1;
    lock.unlock();
    myRobot->lock();
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: Robot is at (%f, %f, %f), GPS (%f, %f) * * * *", myRobot->getX(), myRobot->getY(), myRobot->getTh(), myGPS->getLatitude(), myGPS->getLongitude()); 
    myRobot->unlock();
    lock.lock();
    if(!getRunning()) return 0;
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: telling action to let navigation resume...", myTime);
    myAction->resume();
    return 0;
  }

  void handlePopup(ArTypes::Byte4 popupID, int button)
  {
    ScopedLock lock(mutex);
     if(button == 0) //OK
        keepPausing = false;
     else if(button == 1 || button == -2) // Ignore or closed
        popupServer->closePopup(popupID, "Ignore");
  }

};

#endif

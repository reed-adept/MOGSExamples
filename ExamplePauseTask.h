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

// Example target of RegularStopAction
class ExamplePauseTask : public virtual ArASyncTask
{
  ArRetFunctorC<bool, ExamplePauseTask> myStartCallback;
  double myTime;
  ArRobot *myRobot;
  RegularStopAction *myAction;
  bool keepPausing;
  ArServerHandlerPopup *popupServer;
  ArServerHandlerPopupInfo popupInfo;
  ArFunctor2C<ExamplePauseTask, ArTypes::Byte4, int> handlePopupCB;

public:
  ExamplePauseTask(double time, ArRobot *robot, RegularStopAction *action, ArServerHandlerPopup *_popupServer = NULL) :
    myStartCallback(this, &ExamplePauseTask::start), 
    myTime(time), myRobot(robot), myAction(action),
    popupServer(_popupServer),
    keepPausing(false),
    popupInfo(NULL, "Example Pause Task", "Pausing...", ArServerHandlerPopup::INFORMATION, 1, 0, -1, NULL, "Resume", "Resuming...", "Ignore", "Ignoring..."),
    handlePopupCB(this, &ExamplePauseTask::handlePopup)
  {
    Aria::getConfig()->addParam(ArConfigArg("PauseTime", &myTime, "Time to pouse and do nothing before end of task (secs)", 0.0), "Pause Task");
  }

  ArRetFunctor<bool> * getStartCallback() { return &myStartCallback; }


protected:
  bool start() 
  {
    runAsync();
    return false;
  }

  void *runThread(void*) 
  {
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: started, waiting %.0f ms...", myTime*1000);
    ArTime sleepTime;
    ArTypes::Byte4 popupID = 0;
    if(popupServer) popupID = popupServer->createPopup(&popupInfo, &handlePopupCB);
    keepPausing = true;
    while(keepPausing)
    {
	if(sleepTime.secSince() >= myTime)
        	break;
        ArUtil::sleep(200);
    }
    keepPausing = false;
    if(popupServer) popupServer->closePopup(popupID, "Resuming");
    myRobot->lock();
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: Robot is at %f, %f, %f * * * *", myRobot->getX(), myRobot->getY(), myRobot->getTh()); 
    myRobot->unlock();
    ArLog::log(ArLog::Normal, " * * * * ExamplePauseTask: telling action to let navigation resume...", myTime);
    myAction->resume();
    return 0;
  }

  void handlePopup(ArTypes::Byte4 popupID, int button)
  {
printf("BUTTON %d\n", button);
     if(button == 0) //OK
        keepPausing = false;
     else if(button == 1 || button == -2) // Ignore or closed
        popupServer->closePopup(popupID, "Ignore");
  }

};

#endif

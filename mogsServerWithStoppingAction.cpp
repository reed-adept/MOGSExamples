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

/*
Copyright (c) 2014-2016 Adept Technology Inc.
All rights reserved.Â  

Redistribution of this example source code, with or without modification, is 
permitted provided that the following conditions are met:Â Â  
-Â Â Â  Redistributions must retain the above copyright notice, 
     this list of conditions and the following disclaimer.Â  
-Â  Â  Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Adept Technology, Inc.

Adept Technology, Inc. makes no warranty as to the suitability of this material
for use by the recipient, and assumes no responsibility for any consequences
resulting from such use. 

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/

/** @example mogsServer.cpp example of almost all MOGS features */


/**
  This is an example of using ARNL and ArNetworking to provide a
  server program that remote clients, such as MobileEyes, can connect
  to, and which includes most of the features available in ARNL.

  You can make a copy of this program's source file in the 'examples' directory
  and use it as the basis for your own program.

  MobileEyes sends requests to this server program to go to goals,
  get the current map, set configuration parameters, manually relocalize, 
  request the current state of the robot and sensors, etc. This example server
  includes almost all features provided by ARNL (or SONARNL or MOGS), including
  localization, path planning and navigation to a map goal or abritrary point,
  sharing information with other servers peer-to-peer or through 
  a "central server", using IR and bumper sensors, stopping the robot
  if localization fails, global replanning if a path is not followable,
  use of special SICK laser reflector beacons, as well as various networking
  services for MobileEyes and other clients, such as diagnostic visualizations,
  access to configuration parameters, access to camera control and 
  video images (if SAVServer is running), special debugging commands 
  ("custom commands"), file uploading/downloading (Linux only) and gathering raw laser
  scans using Mapper3, and safe and unsafe teleoperation.

  When this program runs, ARNL will load its configuration file arnl.p. This
  file may specify the map file to use; if arnl.p does not yet have a map
  file name saved or has the wrong map, a map may be set using MobileEyes
  (Open Tools->Robot Configuration, and change the Map in the Files section).
 
  Note: This program creates an ArPoseStorage object which periodically saves 
  the robot position to a file called robotPose. On startup, the robot's 
  position (pose) in ARIA is set to this position, and if using the simulator, 
  the robot is moved to that position in the simulator.  ARNL will then use 
  that as a starting point when doing the initial localization (in
  localizeRobotAtHomeBlocking()) as well as any home points in the map.
  You can delete the 'robotPose' file to reset this behavior (then 0,0,0 is
  assumed to be the robot's initial position, and this is tried during initial
  localization as well as the home points).
*/

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArGPSLocalizationTask.h"
#include "ArSystemStatus.h"

#include "GPSMapTools.h"
#include "ExamplePauseTask.h"
#include "RegularStopAction.h"

void logOptions(const char *progname)
{
  ArLog::log(ArLog::Normal, "Usage: %s [options]\n", progname);
  ArLog::log(ArLog::Normal, "[options] are any program options listed below, or any ARNL configuration");
  ArLog::log(ArLog::Normal, "parameters as -name <value>, see params/arnl.p for list.");
  ArLog::log(ArLog::Normal, "For example, -map <map file>.");
  Aria::logOptions();
}

bool gyroErrored = false;
const char* getGyroStatusString(ArRobot* robot)
{
  if(!robot || !robot->getOrigRobotConfig() || robot->getOrigRobotConfig()->getGyroType() < 2) return "N/A";
  if(robot->getFaultFlags() & ArUtil::BIT4)
  {
    gyroErrored = true;
    return "ERROR/OFF";
  }
  if(gyroErrored)
  {
    return "OK but error before";
  }
  return "OK";
}

// This function is called from the ArRobot task thread on every robot update,
// which is every 100ms.  You can check timers here and do any non-blocking
// monitoring of I/O.  Any long-blocking operations must not be done in this
// callback however, or it will impact the ArRobot task thread and
// communications with the robot (use ArASyncTask to run a new thread instead.)
void robotTaskExample(ArRobot* robot)
{
}

// This function is called whenever a new goal is reached. It will be attached 
// to the path planning task below in main() via ArPathPlanningTask::addGoalDoneCB()
void goalDone(ArPose goalPos) //, ArPathPlanningTask *pathTask)
{
  ArLog::log(ArLog::Normal, "ARNL server example: goal reached");
}

// This function is called if pathplanning fails. It will be attached 
// to the path planning task below in main() via
// ArPathPlanningTask::addGoalFailedCB()
void goalFailed(ArPose goalPos) //, ArPathPlanningTask *pathTask)
{
  ArLog::log(ArLog::Normal, "ARNL server example: goal failed");
}

// This function is called if localization fails. It will be attached 
// to the localization task below in main() via
// ArLocalizationTask::setFailedCallback()
void locFailed(int n) //, ArLocalizationTask* locTask)
{
  ArLog::log(ArLog::Normal, "ARNL server example: localization failed");
}
 
// This function is called whenever the path planning task changes its state
// (for example, from idle to planning a path, to following a planned path). It will be attached 
// to the path planning task below in main() via
// ArPathPlanningTask::addStateChangeCB()
//ArPathPlanningInterface *pathPlanningTask = NULL;
void pathPlanStateChanged(ArPathPlanningTask *pathPlanningTask)
{
  char s[256];
  pathPlanningTask->getFailureString(s, 256);
  ArLog::log(ArLog::Normal, "ARNL server example: Path planning state: %s", s);
}


// This class sends a warning message (popup) to a client if the robot's motors
// become disabled, with the option to re-enable them. This is neccesary on some
// robots (e.g. Pioneer LX) for the user to re-enable motors after e-stop.
// This class also changes the state of the Pioneer LX wheel lights based on 
// e-stop and motor power conditions. (You can customize what the wheel lights
// do by modifying this or sending similar commands based on other conditions
// in your software.)
class RobotMonitor
{
protected:
  ArRobot *robot;
  ArServerHandlerPopup *popupServer;
  ArTypes::Byte4 motorsDisabledPopupID;
  ArServerHandlerPopupInfo motorsDisabledPopupInfo;
  ArFunctor2C<RobotMonitor, ArTypes::Byte4, int> handleMotorsDisabledPopupResponseCB;
  ArFunctorC<RobotMonitor> robotMonitorCB;

public:
  RobotMonitor(ArRobot *r, ArServerHandlerPopup *ps) :
    robot(r),
    popupServer(ps),
    motorsDisabledPopupID(0),
    motorsDisabledPopupInfo( NULL,
      "Robot Motors Are Disabled",
      "The robot's motors are disabled", 
      ArServerHandlerPopup::WARNING,
      1,  // default button ID
      0,  // escape button ID
      -1, // timeout
      NULL, // timeout message
      "Enable Motors", "Enabling Motors..." //, "Ignore", "Ignore"
    ),
    handleMotorsDisabledPopupResponseCB(this, &RobotMonitor::handleMotorsDisabledResponse),
    robotMonitorCB(this, &RobotMonitor::robotMonitorTask)
  {
    robot->addUserTask("arnlServerRobotMonitor", 30, &robotMonitorCB);
  }

  ~RobotMonitor()
  {
    robot->remUserTask(&robotMonitorCB);
  }

protected:
  void handleMotorsDisabledResponse(ArTypes::Byte4 popupID, int button)
  {
    if(button == 0)
    {
      ArLog::log(ArLog::Normal, "Enabling motors...");
      robot->enableMotors();
    }
    popupServer->closePopup(motorsDisabledPopupID, "Closing motor disable popup.");
    motorsDisabledPopupID = 0;
  }


  // This function is called as a robot task (every 100ms) to check on the robot
  // state and perform feedback and interact with user as needed.
  void robotMonitorTask()
  {

    // a way for user to re-enable motors if disabled -- show a popup dialog in
    // MobileEyes.
    if(motorsDisabledPopupID == 0 && robot && !robot->areMotorsEnabled() && robot->isConnected())
    {
      motorsDisabledPopupID = popupServer->createPopup(&motorsDisabledPopupInfo, &handleMotorsDisabledPopupResponseCB);
    }

    // Set LX wheel light pattern based on robot activity. You could add more
    // conditions/light patterns here if you want.
    if(robot->isEStopPressed())
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x02\0\0\0", 4); // pattern #2, flash red
    else if(!robot->areMotorsEnabled())
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x03\0\0\0", 4); // pattern #3, flash yellow
    else if(fabs(robot->getVel()) < 5)
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x0A\0\0\0", 4);  // pattern #10, slow blue flash
    else
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x09\0\0\0", 4);  // pattern 9, blue sweep.
    
  }
};

// Adds some "custom commands" to switch robot power outputs on/off
class SimpleComSwitchPower
{
protected:
  ArMTXIO mtxio;
	ArRobot *robot;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> mtxOnCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> mtxOffCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> seekurOnCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> seekurOffCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> patrolbotOnCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> patrolbotOffCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> resetCB;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> *onCmd;
  ArFunctor1C<SimpleComSwitchPower, ArArgumentBuilder*> *offCmd;
  void switchMTX(ArArgumentBuilder *args, bool state, const char *name = NULL);
  void mtxOn(ArArgumentBuilder *args);
  void mtxOff(ArArgumentBuilder *args);
  void seekurOn(ArArgumentBuilder *args);
  void seekurOff(ArArgumentBuilder *args);
  void patrolbotOn(ArArgumentBuilder *args);
  void patrolbotOff(ArArgumentBuilder *args);
  void reset(ArArgumentBuilder *args);
public:
  SimpleComSwitchPower(ArServerHandlerCommands *commands, ArRobot *robot);
};


SimpleComSwitchPower::SimpleComSwitchPower(ArServerHandlerCommands *commands, ArRobot *_robot) :
  robot(_robot),
  mtxOnCB(this, &SimpleComSwitchPower::mtxOn),
  mtxOffCB(this, &SimpleComSwitchPower::mtxOff),
  seekurOnCB(this, &SimpleComSwitchPower::seekurOn), 
  seekurOffCB(this, &SimpleComSwitchPower::seekurOff), 
  patrolbotOnCB(this, &SimpleComSwitchPower::patrolbotOn), 
  patrolbotOffCB(this, &SimpleComSwitchPower::patrolbotOff), 
  resetCB(this, &SimpleComSwitchPower::reset),
  onCmd(NULL),
  offCmd(NULL)
{
  if(mtxio.isEnabled())
  {
    onCmd = &mtxOnCB;
    offCmd = &mtxOffCB;
  }
  else if(robot && (
    strcasecmp(robot->getRobotSubType(), "researchPB") == 0 ||
    strcasecmp(robot->getRobotSubType(), "patrolbot-sh") == 0 ||
    strcasecmp(robot->getRobotSubType(), "mt400") == 0 ||
    strcasecmp(robot->getRobotSubType(), "guiabot") == 0 ||
    strcasecmp(robot->getRobotSubType(), "mt490") == 0)
  )
  {
    onCmd = &patrolbotOnCB;
    offCmd = &patrolbotOffCB;
  }
  else if(robot && (
    strcasecmp(robot->getRobotSubType(), "seekur") == 0 ||
    strcasecmp(robot->getRobotSubType(), "seekurjr") == 0)
  )
  {
    onCmd = &seekurOnCB;
    offCmd = &seekurOffCB; 
  }
  else
  {
    return;
  }

  
  commands->addStringCommand("PowerOutputOn",
    "Switch on the given power output. For PioneerLX/MTX, specify {aux5v|aux12v|aux20v|user1|user2|user3} or <bank> <bit>.  For Seekur, specify <port>.  For PatrolBot, specify <port>.",
    onCmd
  );
  commands->addStringCommand("PowerOutputOff",
    "Switch off the given power output. For PioneerLX/MTX, specify {aux5v|aux12v|aux20v|user1|user2|user3} or <bank> <bit>.  For Seekur, specify <port>.  For PatrolBot, specify <port>.",
    offCmd
  );
  commands->addStringCommand("PowerOutputReset",
    "Toggle the given power output off, then on after a short delay. For PioneerLX/MTX, specify {aux5v|aux12v|aux20v|user1|user2|user3} or <bank> <bit>.  For Seekur, specify <port>.  For PatrolBot, specify <port>.",
    &resetCB
  );
}

void SimpleComSwitchPower::reset(ArArgumentBuilder *args)
{
  offCmd->invoke(args);
  ArUtil::sleep(1000);
  onCmd->invoke(args);
}

void SimpleComSwitchPower::mtxOn(ArArgumentBuilder *args)
{
  switchMTX(args, true);
}

void SimpleComSwitchPower::mtxOff(ArArgumentBuilder *args)
{
  switchMTX(args, false);
}

void SimpleComSwitchPower::switchMTX(ArArgumentBuilder *args, bool state, const char *name)
{
  int bank = 0;
  int bit = 0;
  if(args->getArgc() == 1)
  {
    if(strcasecmp(args->getArg(0), "aux5v") == 0)
    {
      bank = 3; bit = 1;
    }
    else if(strcasecmp(args->getArg(0), "aux12v") == 0)
    {
      bank = 3; bit = 2;
    }
    else if(strcasecmp(args->getArg(0), "aux20v") == 0)
    {
      bank = 3; bit = 3;
    }
    else if(strcasecmp(args->getArg(0), "user1") == 0)
    {
      bank = 2; bit = 5;
    }
    else if(strcasecmp(args->getArg(0), "user2") == 0)
    {
      bank = 2; bit = 6;
    }
    else if(strcasecmp(args->getArg(0), "user3") == 0)
    {
      bank = 2; bit = 7;
    }
    else
    {
      ArLog::log(ArLog::Normal, "SimpleComSwitchPower: error: unrecognized name with PowerOutput command for LX/MTX. Must be one of: aux5v, aux12v, aux20v, user1, user2, user3.") ;
      return;
    }
    name = args->getArg(0);
  }
  else if(args->getArgc() == 2)
  {
    bank = args->getArgInt(0);
    bit = args->getArgInt(1);
  }
  else
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: error: must give name or two arguments with PowerOutput command for LX/MTX.") ;
    return;
  }
  if(!mtxio.isEnabled())
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: error: not connected to MTX IO.");
    return;
  }
  if(name)
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: switching MTX %s (%d,%d) %s.", name, bank, bit, state?"on":"off");
  else
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: switching MTX IO bank %d bit %d %s.", bank, bit, state?"on":"off");
  if(!mtxio.setPowerOutput(bank-1, bit, state))
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: error setting power output.");
  }
}


void SimpleComSwitchPower::seekurOn(ArArgumentBuilder *args)
{
  if(args->getArgc() != 1)
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: no argument given with PowerOutput command.") ;
    return;
  }
  int p = args->getArgInt(0);
  robot->lock();
  ArLog::log(ArLog::Normal, "SimpleComSwitchPower: Switching Seekur power port %d on.", p);
  robot->com2Bytes(116, p, 1);
  robot->unlock();
}

void SimpleComSwitchPower::seekurOff(ArArgumentBuilder *args)
{
  if(args->getArgc() != 1)
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: no argument given with PowerOutput command."); 
    return;
  }
  int p = args->getArgInt(0);
  robot->lock();
  ArLog::log(ArLog::Normal, "SimpleComSwitchPower: Switching Seekur power port %d off.", p);
  robot->com2Bytes(116, p, 0);
  robot->unlock();
}

void SimpleComSwitchPower::patrolbotOn(ArArgumentBuilder *args)
{
  if(args->getArgc() != 1)
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: no argument given with PowerOutput command.") ;
    return;
  }
  int p = args->getArgInt(0);
  robot->lock();
  ArLog::log(ArLog::Normal, "SimpleComSwitchPower: Switching PatrolBot power port %d on.", p);
  robot->com2Bytes(31, p, 1);
  robot->unlock();
}

void SimpleComSwitchPower::patrolbotOff(ArArgumentBuilder *args)
{
  if(args->getArgc() != 1)
  {
    ArLog::log(ArLog::Normal, "SimpleComSwitchPower: no argument given with PowerOutput command."); 
    return;
  }
  int p = args->getArgInt(0);
  robot->lock();
  ArLog::log(ArLog::Normal, "SimpleComSwitchPower: Switching PatrolBot power port %d off.", p);
  robot->com2Bytes(31, p, 0);
  robot->unlock();
}

/// Log messages from robot controller
bool handleDebugMessage(ArRobotPacket *pkt)
{
  if(pkt->getID() != ArCommands::MARCDEBUG) return false;
  char msg[256];
  pkt->bufToStr(msg, sizeof(msg));
  msg[255] = 0;
  ArLog::log(ArLog::Terse, "Controller Firmware: %s", msg);
  return true;
}



// an example function to call at each regular stop. could be in a class if you
// don't want to use global ArRobot*. Must return immediately.
ArRobot *RobotPtr;
void printRobotPos()
{
  ArLog::log(ArLog::Normal, " * * * * Robot is at %f, %f, %f * * * *", RobotPtr->getX(), RobotPtr->getY(), RobotPtr->getTh()); 
}




// Program main function.
int main(int argc, char **argv)
{
  // Initialize Aria and Arnl global information
  Aria::init();
  Arnl::init();

  // You can change default ArLog options in this call, but the settings in the parameter file
  // (arnl.p) which is loaded below (Aria::getConfig()->parseFile())  will override the options.
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);

  // Used to parse the command line arguments.
  ArArgumentParser parser(&argc, argv);
  
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // Tell the laser connector to always connect the first laser since
  // this program always requires a laser.
  parser.addDefaultArgument("-connectLaser");

  


  // The robot object
  ArRobot robot;

  // This is an example of a user task callback. This is called every ArRobot
  // task cycle (every 100ms). The function is defined at the top of this file.
  ArGlobalFunctor1<ArRobot*> robotTaskExampleCB(&robotTaskExample, &robot);
  robot.addUserTask("arnlServerExampleTask", 10, &robotTaskExampleCB);

  // handle messages from robot controller firmware and log the contents
  robot.addPacketHandler(new ArGlobalRetFunctor1<bool, ArRobotPacket*>(&handleDebugMessage));

  // This object is used to connect to the robot, which can be configured via
  // command line arguments.
  ArRobotConnector robotConnector(&parser, &robot);

  // Connect to the robot
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to robot... exiting");
    Aria::exit(3);
  }



  // Set up where we'll look for files. Arnl::init() set Aria's default
  // directory to Arnl's default directory; addDirectories() appends this
  // "examples" directory.
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  
  
  // To direct log messages to a file, or to change the log level, use these  calls:
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
  //ArLog::init(ArLog::File, ArLog::Verbose);
 
  // Add a section to the configuration to change ArLog parameters
  ArLog::addToConfig(Aria::getConfig());

  // set up a gyro (if the robot is older and its firmware does not
  // automatically incorporate gyro corrections, then this object will do it)
  ArAnalogGyro gyro(&robot);

  // Our networking server
  ArServerBase server;
  
  // GPS connector.
  ArGPSConnector gpsConnector(&parser);

  // Set up our simpleOpener, used to set up the networking server
  ArServerSimpleOpener simpleOpener(&parser);

  // the laser connector
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  // used to connect to camera PTZ control
  ArPTZConnector ptzConnector(&parser, &robot);

  
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // Parse arguments 
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(1);
  }
  

  // This causes Aria::exit(9) to be called if the robot unexpectedly
  // disconnects
  ArGlobalFunctor1<int> shutdownFunctor(&Aria::exit, 9);
  robot.addDisconnectOnErrorCB(&shutdownFunctor);


  // Create an ArSonarDevice object (ArRangeDevice subclass) and 
  // connect it to the robot.
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);



  // This object will allow robot's movement parameters to be changed through
  // a Robot Configuration section in the ArConfig global configuration facility.
  ArRobotConfig robotConfig(&robot);

  // Include gyro configuration options in the robot configuration section.
  robotConfig.addAnalogGyro(&gyro);

  // Start the robot thread.
  robot.runAsync(true);

  // On the Seekur, power to the GPS receiver is switched on by this command.
  // (A third argument of 0 would turn it off). On other robots this command is
  // ignored. If this fails, you may need to reset the port with ARIA demo or 
  // seekurPower program (turn port off then on again).  If the port is already
  // on, it will have no effect on the GPS (it will remain powered.)
  // Do this now before connecting to lasers to give it plenty of time to power
  // on, initialize, and find a good position before GPS localization begins.
  ArLog::log(ArLog::Normal, "Turning on GPS power... (Seekur/Seekur Jr. power port 6)");
  robot.com2Bytes(116, 6, 1);
  

  // connect the laser(s) if it was requested, this adds them to the
  // robot too, and starts them running in their own threads
  ArLog::log(ArLog::Normal, "Connecting to laser(s) configured in parameters...");
  if (!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to laser(s). Exiting.");
    Aria::exit(2);
  }
  ArLog::log(ArLog::Normal, "Done connecting to laser(s).");

  // find the laser we should use for localization and/or mapping,
  // which will be the first laser
  robot.lock();
  ArLaser *firstLaser = robot.findLaser(1);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ArLog::log(ArLog::Normal, "Did not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting");
    Aria::exit(2);
  }
  robot.unlock();


    /* Create and set up map object */
  
  // Set up the map object, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the program's current directory
  // instead.
  // When a configuration file is loaded into ArConfig later, if it specifies a
  // map file, then that file will be loaded as the map.
  ArMap map(fileDir);
  // set it up to ignore empty file names (otherwise if a configuration omits
  // the map file, the whole configuration change will fail)
  map.setIgnoreEmptyFileName(true);
  // ignore the case, so that if someone is using MobileEyes or
  // MobilePlanner from Windows and changes the case on a map name,
  // it will still work.
  map.setIgnoreCase(true);

    
    /* Create localization and path planning threads */


  ArPathPlanningTask pathTask(&robot, &sonarDev, &map);
//  pathPlanningTask = &pathTask;

  // The following are callback functions that pathTask will call 
  // on certain events such as reaching a goal, failing to reach a goal, etc.
  // The functions are defined at the top of this file. You can add code there
  // to take some action such as control something on the robot, choose the next
  // goal, send output elsewhele, etc.
  ArGlobalFunctor1<ArPose> /*, ArPathPlanningTask*>*/ goalDoneCB(&goalDone); //, &pathTask);
  pathTask.addGoalDoneCB(&goalDoneCB);

  ArGlobalFunctor1<ArPose> /*, ArPathPlanningTask*>*/ goalFailedCB(&goalFailed); //, &pathTask);
  pathTask.addGoalFailedCB(&goalFailedCB);

  ArGlobalFunctor1<ArPathPlanningTask*> stateCB(&pathPlanStateChanged, &pathTask);
  pathTask.addStateChangeCB(&stateCB);


  



  ArLog::log(ArLog::Normal, "Connecting to GPS...");

  // Connect to GPS
  ArGPS *gps = gpsConnector.createGPS(&robot);
  if(!gps || !gps->connect())
  {
    ArLog::log(ArLog::Terse, "Error connecting to GPS device."
      "Try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments."
      "Use -help for help. Exiting.");
    Aria::exit(5);
  }

  // set up GPS localization task
  ArLog::log(ArLog::Normal, "Creating GPS localization task");
  ArGPSLocalizationTask gpsLocTask(&robot, gps, &map);

  // Set some options  and callbacks on each laser that the laser connector 
  // connected to.
  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot.getLaserMap()->begin();
       laserIt != robot.getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    // Skip lasers that aren't connected
    if(!laser->isConnected())
      continue;

    // add the disconnectOnError CB to shut things down if the laser
    // connection is lost
    laser->addDisconnectOnErrorCB(&shutdownFunctor);
    // set the number of cumulative readings the laser will take
    laser->setCumulativeBufferSize(200);
    // add the lasers to the path planning task
    pathTask.addRangeDevice(laser, ArPathPlanningTask::BOTH);
    // set the cumulative clean offset (so that they don't all fire at once)
    laser->setCumulativeCleanOffset(laserNum * 100);
    // reset the cumulative clean time (to make the new offset take effect)
    laser->resetLastCumulativeCleanTime();

    // Add the packet count to the Aria info strings (It will be included in
    // MobileEyes custom details so you can monitor whether the laser data is
    // being received correctly)
    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10, 
	    new ArRetFunctorC<int, ArLaser>(laser, 
					 &ArLaser::getReadingCount));
  }





    /* Start the server */

  // Open the networking server
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    ArLog::log(ArLog::Normal, "Error: Could not open server.");
    exit(2);
  }



    /* Create various services that provide network access to clients (such as
     * MobileEyes), as well as add various additional features to ARNL */





  /* Add additional range devices to the robot and path planning task (so it
     avoids obstacles detected by these devices) */
  
  // Add IR range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  robot.lock();
  ArIRs irs;
  robot.addRangeDevice(&irs);
  pathTask.addRangeDevice(&irs, ArPathPlanningTask::CURRENT);

  // Add bumpers range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);
  pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Add range device which uses forbidden regions given in the map to give virtual
  // range device readings to ARNL.  (so it avoids obstacles
  // detected by this device)
  ArForbiddenRangeDevice forbidden(&map);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  robot.unlock();


  // Action to slow down robot when localization score drops but not lost.
  ArActionSlowDownWhenNotCertain actionSlowDown(&gpsLocTask);
  pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

  // Action to stop the robot when localization is "lost" (score too low)
  ArActionLost actionLostPath(&gpsLocTask, &pathTask);
  pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

  // Arnl uses this object when it must replan its path because its
  // path is completely blocked.  It will use an older history of sensor
  // readings to replan this new path.  This should not be used with SONARNL
  // since sonar readings are not accurate enough and may prevent the robot
  // from planning through space that is actually clear.
  ArGlobalReplanningRangeDevice replanDev(&pathTask);

  
  // Service to provide drawings of data in the map display :
  ArServerInfoDrawings drawings(&server);
  drawings.addRobotsRangeDevices(&robot);
  drawings.addRangeDevice(&replanDev);

  /* Draw a box around the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) */
  ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorP(&pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP); 

  /* Draw a box showing the safety clearance used by the path planning task to */
  drawings.addDrawing(
    new ArDrawingData("polySegments", ArColor(166, 166, 166), 1, 60, 100, "DefaultOn"),
    "Path Planning Clearances",
    new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(&pathTask, &ArPathPlanningTask::drawRobotBounds)
  );


  /* Show the positions calculated by GPS localization */

  ArDrawingData drawingDataG("polyDots", ArColor(100,100,255), 130, 61);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG(&gpsLocTask, &ArGPSLocalizationTask::drawGPSPoints);
  drawings.addDrawing(&drawingDataG, "GPS Points", &drawingFunctorG);

  ArDrawingData drawingDataG2("polyDots", ArColor(255,100,100), 100, 62);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG2(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanPoints);
  drawings.addDrawing(&drawingDataG2, "Kalman Points", &drawingFunctorG2);

  ArDrawingData drawingDataG3("polyDots", ArColor(100,255,100), 70, 63);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG3(&gpsLocTask, &ArGPSLocalizationTask::drawOdoPoints);
  drawings.addDrawing(&drawingDataG3, "Odom. Points", &drawingFunctorG3);

  ArDrawingData drawingDataG4("polyDots", ArColor(255,50,50), 100, 75);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG4(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanRangePoints);
  drawings.addDrawing(&drawingDataG4, "KalRange Points", &drawingFunctorG4);

  ArDrawingData drawingDataG5("polySegments", ArColor(100,0,255), 1, 78);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG5(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanVariance);
  drawings.addDrawing(&drawingDataG5, "VarGPS", &drawingFunctorG5);

  // "Custom" commands. You can add your own custom commands here, they will
  // be available in MobileEyes' custom commands (enable in the toolbar or
  // access through Robot Tools)
  ArServerHandlerCommands commands(&server);


  // These provide various kinds of information to the client:
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  serverInfoPath.addSearchRectangleDrawing(&drawings);
  serverInfoPath.addControlCommands(&commands);

  // Provides localization info and allows the client (MobileEyes) to relocalize at a given
  // pose:
  ArServerInfoLocalization serverInfoLocalization(&server, &robot, &gpsLocTask);
  ArServerHandlerLocalization serverLocHandler(&server, &robot, &gpsLocTask);

  // If you're using MobileSim, ArServerHandlerLocalization sends it a command
  // to move the robot's true pose if you manually do a localization through 
  // MobileEyes.  To disable that behavior, use this constructor call instead:
  // ArServerHandlerLocalization serverLocHandler(&server, &robot, true, false);
  // The fifth argument determines whether to send the command to MobileSim.

  // Provide the map to the client (and related controls):
  ArServerHandlerMap serverMap(&server, &map);

  // These objects add some simple (custom) commands to 'commands' for testing and debugging:
  ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
//  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


  // service that allows the client to monitor the communication link status
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor handlerCommMonitor(&server);



  // service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  // This service causes the client to show simple dialog boxes
  ArServerHandlerPopup popupServer(&server);

  // Monitor the robot for current state such as if the motors are disabled,
  // indicate this state by various means and show a popup dialog on clients
  // if user confirmation is needed.
  // (Namely, some robots  disable motors automatically on various error conditions,
  // and we want the user to manually re-enable them after resolving the
  // problem)
  RobotMonitor robotMonitor(&robot, &popupServer);

  
  /* Add an action to the path planning action group that will cause the robot
   * to stop at a distance interval, and perform some task.  This could be a
   * simple function call to printRobotPos() that returns immediately, or it could
   * start a longer running task ExamplePauseTask.
    */
  ArRetFunctor<bool> *stopCB = NULL;

  // Simple immediate function call:
  //RobotPtr = &robot;
  //GlobalFixedRetFunctor<bool> printRobotPosCB(&printRobotPos, true);
  //stopCB = &printRobotPosCB;
  

  // Create the RegularStopAction and add it to the path planning action group:
  RegularStopAction regularStopAction(3000/*mm*/, "RegularStopAction", &commands);
  pathTask.getPathPlanActionGroup()->addAction(&regularStopAction, 75);


  // Longer running asynchronous task:
  ExamplePauseTask exampleTask(5.0/*sec*/, &robot, gps, &regularStopAction, &popupServer);
  

  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
    ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map,
			    gpsLocTask.getRobotHome(),
			    gpsLocTask.getRobotHomeCallback());

  // Add a special command to Custom Commands that tours a list of goals rather
  // than all:
  modeGoto.addTourGoalsInListSimpleCommand(&commands);

  // Mode To stop and remain stopped:
  ArServerModeStop modeStop(&server, &robot);

  // Cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, if using SONARNL to localize, then don't do this
  // since localization may get lost)
  ArSonarAutoDisabler sonarAutoDisabler(&robot);

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
//  ArServerModeDrive modeDrive(&server, &robot);            // Older mode for compatability



  // Prevent normal teleoperation driving if localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost actionLostRatioDrive(&gpsLocTask, &pathTask, &modeRatioDrive);
  modeRatioDrive.getActionGroup()->addAction(&actionLostRatioDrive, 110);

  // Add drive mode section to the configuration, and also some custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive.addControlCommands(&commands);

  // Wander mode (also prevent wandering if lost):
  ArServerModeWander modeWander(&server, &robot);
  ArActionLost actionLostWander(&gpsLocTask, &pathTask, &modeWander);
  modeWander.getActionGroup()->addAction(&actionLostWander, 110);

  // Tool to log data periodically to a file
  ArDataLogger dataLogger(&robot, "datalog.txt");
  dataLogger.addToConfig(Aria::getConfig()); // make it configurable through ArConfig

  // Automatically add anything from the global info group to the data logger.
  Aria::getInfoGroup()->addAddStringCallback(dataLogger.getAddStringFunctor());

  // This provides a small table of interesting information for the client
  // to display to the operator. You can add your own callbacks to show any
  // data you want.
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  
  // The following statements add fields to a set of informational data called
  // the InfoGroup. These are served to MobileEyes for displayi (turn on by enabling Details
  // and Custom Details in the View menu of MobileEyes.)

  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));




  const char *dopfmt = "%2.4f";
  const char *posfmt = "%2.8f";
  const char *altfmt = "%3.6f m";
  Aria::getInfoGroup()->addStringString(
	    "GPS Fix Mode", 25,
	    new ArConstRetFunctorC<const char*, ArGPS>(gps, &ArGPS::getFixTypeName)
    );
  Aria::getInfoGroup()->addStringInt(
	    "GPS Num. Satellites", 4,
	    new ArConstRetFunctorC<int, ArGPS>(gps, &ArGPS::getNumSatellitesTracked)
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS HDOP", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getHDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS VDOP", 5,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getVDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS PDOP", 5,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getPDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Latitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Longitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Altitude", 8,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitude),
      altfmt
    );

  // only some GPS receivers provide these, but you can uncomment them
  // here to enable them if yours does.
  /*
  const char *errfmt = "%2.4f m";
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lat. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lon. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Alt. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitudeError),
      errfmt
    );
  */

  Aria::getInfoGroup()->addStringDouble(
    "GPS Loc Score", 8,
    new ArRetFunctorC<double, ArGPSLocalizationTask>(
      &gpsLocTask, &ArGPSLocalizationTask::getLocalizationScore),
    "%.03f"
  );

  Aria::getInfoGroup()->addStringBool(
    "GPS Loc Lost", 8,
    new ArRetFunctorC<bool, ArGPSLocalizationTask>(
      &gpsLocTask, &ArGPSLocalizationTask::getRobotIsLostFlag)
  );

  Aria::getInfoGroup()->addStringBool(
    "GPS Loc Idle", 8,
    new ArRetFunctorC<bool, ArGPSLocalizationTask>(
      &gpsLocTask, &ArGPSLocalizationTask::getIdleFlag)
  );


  // Display gyro status if gyro is enabled and is being handled by the firmware (gyro types 2, 3, or 4).
  // (If the firmware detects an error communicating with the gyro or IMU it
  // returns a flag, and stops using it.)
  // (This gyro type parameter, and fault flag, are only in ARCOS, not Seekur firmware)
  if(robot.getOrigRobotConfig() && robot.getOrigRobotConfig()->getGyroType() > 1)
  {
    Aria::getInfoGroup()->addStringString(
          "Gyro/IMU Status", 10,
          new ArGlobalRetFunctor1<const char*, ArRobot*>(&getGyroStatusString, &robot)
      );
  }

  Aria::getInfoGroup()->addStringInt("Temp. (deg C)", 10, new ArConstRetFunctorC<int, ArRobot>(&robot, &ArRobot::getTemperature), "%d"); 
  // Display system CPU and wireless network status
  ArSystemStatus::startPeriodicUpdate(1000); // update every 1 second
  Aria::getInfoGroup()->addStringDouble("CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
  Aria::getInfoGroup()->addStringInt("Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Link Noise", 9, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Signal", 9, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
  

  // stats on how far its driven since software started
  Aria::getInfoGroup()->addStringDouble("Distance Travelled (m)", 20, new ArRetFunctorC<double, ArRobot>(&robot, &ArRobot::getOdometerDistanceMeters), "%.2f");
  Aria::getInfoGroup()->addStringDouble("Run time (min)", 20, new
ArRetFunctorC<double, ArRobot>(&robot, &ArRobot::getOdometerTimeMinutes),
"%.2f");




  // Add some "custom commands" for setting up initial GPS offset and heading.
  gpsLocTask.addLocalizationInitCommands(&commands);
  
  // Add some commands for manually creating map objects based on GPS positions:
//  ArGPSMapTools gpsMapTools(gps, &robot, &commands, &map);

  // Add command to set simulated GPS position manually
  if(gpsConnector.getGPSType() == ArGPSConnector::Simulator)
  {
    ArSimulatedGPS *simGPS = dynamic_cast<ArSimulatedGPS*>(gps);
//    simGPS->setDummyPosition(42.80709, -71.579047, 100);
    commands.addStringCommand("GPS:setDummyPosition", 
      "Manually set a new dummy position for simulated GPS. Provide latitude (required), longitude (required) and altitude (optional)", 
      new ArFunctor1C<ArSimulatedGPS, ArArgumentBuilder*>(simGPS, &ArSimulatedGPS::setDummyPositionFromArgs)
    );
  }


  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();
    // TODO move up near where stop mode is created?





  /* Services that allow the client to initiate scanning with the laser to
     create maps in Mapper3 (So not possible with SONARNL): */

  ArServerHandlerMapping handlerMapping(&server, &robot, firstLaser, 
					fileDir, "", true);


  // Save GPS positions in the .2d scan log when making a map
  handlerMapping.addLocationData("robotGPS", 
			    gpsLocTask.getPoseInterpPositionCallback());

  // add the starting latitude and longitude info to the .2d scan log
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArGPSLocalizationTask, ArServerHandlerMapping *>
	  (&gpsLocTask, &ArGPSLocalizationTask::addScanInfo, 
	   &handlerMapping));

  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping.addMappingStartCallback(actionLostPath.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostRatioDrive.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostWander.getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping.addMappingEndCallback(actionLostPath.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostRatioDrive.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostWander.getEnableCB());

  // don't let forbidden lines show up as obstacles while mapping
  // (they'll just interfere with driving while mapping, and localization is off anyway)
  handlerMapping.addMappingStartCallback(forbidden.getDisableCB());

  // let forbidden lines show up as obstacles again as usual after mapping
  handlerMapping.addMappingEndCallback(forbidden.getEnableCB());


  /*
  // If we are on a simulator, move the robot back to its starting position,
  // and reset its odometry.
  // This will allow localizeRobotAtHomeBlocking() below will (probably) work (it
  // tries current odometry (which will be 0,0,0) and all the map
  // home points.
  // (Ignored by a real robot)
  //robot.com(ArCommands::SIM_RESET);
  */


  // create a pose storage class, this will let the program keep track
  // of where the robot is between runs...  after we try and restore
  // from this file it will start saving the robot's pose into the
  // file
  ArPoseStorage poseStorage(&robot);
  /// if we could restore the pose from then set the sim there (this
  /// won't do anything to the real robot)... if we couldn't restore
  /// the pose then just reset the position of the robot (which again
  /// won't do anything to the real robot)
  if (poseStorage.restorePose("robotPose"))
    serverLocHandler.setSimPose(robot.getPose());
  //else
 //   robot.com(ArCommands::SIM_RESET);



  /* File transfer services: */
  
#ifdef WIN32
  // Not implemented for Windows yet.
  ArLog::log(ArLog::Normal, "Note, file upload/download services are not implemented for Windows; not enabling them.");
#else
  // This block will allow you to set up where you get and put files
  // to/from, just comment them out if you don't want this to happen
  // /*
  ArServerFileLister fileLister(&server, fileDir);
  ArServerFileToClient fileToClient(&server, fileDir);
  ArServerFileFromClient fileFromClient(&server, fileDir, "/tmp");
  ArServerDeleteFileOnServer deleteFileOnServer(&server, fileDir);
  // */
#endif

    /* Video image streaming, and camera controls (Requires SAVserver or ACTS) */

  // Forward one video stream if either ACTS, ArVideo videoSubServer, 
  // or SAV server are running.
  // ArHybridForwarderVideo allows this program to be separate from the ArVideo
  // library. You could replace videoForwarder and the PTZ connection code below
  // with a call to ArVideo::createVideoServers(), and link the program to the
  // ArVideo library if you want to include video capture in the same program
  // as robot control.
  ArHybridForwarderVideo videoForwarder(&server, "localhost", 7070);
  
  // connect to first configured camera PTZ controls (in robot parameter file and
  // command line options)
  ptzConnector.connect();
  ArServerHandlerCamera *cameraServer = NULL;
  ArCameraCollection cameraCollection;
  ArPTZ *ptz = ptzConnector.getPTZ(0);
  if(ptz)
  {
    ArLog::log(ArLog::Normal, "Connected to PTZ Camera");
    cameraCollection.addCamera("Camera1", ptz->getTypeName(), "Camera", ptz->getTypeName());

    videoForwarder.setCameraName("Camera1");
    videoForwarder.addToCameraCollection(cameraCollection);

    cameraServer = new ArServerHandlerCamera("Camera1", 
		                                           &server, 
					                                     &robot,
					                                     ptz, 
					                                     &cameraCollection);

    // ArServerHandlerCamera has a special feature that continuously moves the
    // camera to look at a position. This is an example of how to use that to 
    // look at the path planning task's next goal via its callbacks. You can 
    // also set the ArServerHandhlerCamera to look at any map position instead.
    pathTask.addNewGoalCB(
	    new ArFunctor1C<ArServerHandlerCamera, ArPose>(
		    cameraServer, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalSetGoal));
    pathTask.addGoalFinishedCB(
	    new ArFunctorC<ArServerHandlerCamera>(
		    cameraServer, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalClearGoal));

  } 

  // Allows client to find any camera servers created above
  ArServerHandlerCameraCollection cameraCollectionServer(&server, &cameraCollection);



    /* Load configuration values, map, and begin! */

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(&parser);

  // Read in parameter files.
  ArLog::log(ArLog::Normal, "Loading config file %s%s into ArConfig...", Aria::getDirectory(), Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Could not load ARNL configuration file. Set ARNL environment variable to use non-default installation director.y");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(6);
  }

  // Warn if there is no map
  if (map.getFileName() == NULL || strlen(map.getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### No map file is set up, you can make a map with the following procedure");
    ArLog::log(ArLog::Normal, "\n   See docs/GPSMapping.txt for instructions on creating a map for GPS localization");
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // Do an initial localization of the robot. ARNL and SONARNL try all the home points
  // in the map, as well as the robot's current odometric position, as possible
  // places the robot is likely to be at startup.   If successful, it will
  // also save the position it found to be the best localized position as the
  // "Home" position, which can be obtained from the localization task (and is
  // used by the "Go to home" network request).
  // MOGS instead just initializes at the current GPS position.
  // (You will stil have to drive the robot so it can determine the robot's
  // heading, however. See GPS Mapping instructions.)
  gpsLocTask.localizeRobotAtHomeBlocking();
  

  // Start the networking server's thread
  server.runAsync();



  // Enable the motors and wait until the robot exits (disconnection, etc.) or this program is
  // canceled.
  robot.enableMotors();
  robot.waitForRunExit();
  Aria::exit(0);
}


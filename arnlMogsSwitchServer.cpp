


/*
Copyright (c) 2014-2015 Adept Technology Inc.
Copyright (c) 2016 Omron Adept Technologies, Inc.
All rights reserved.

Redistribution of this example source code, with or without modification, is
permitted provided that the following conditions are met:  
-    Redistributions must retain the above copyright notice,
     this list of conditions and the following disclaimer. 
-    Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Omron Adept Technologies, Inc.

Omron Adept Technologies, Inc. makes no warranty as to the suitability of this
material for use by the recipient, and assumes no responsibility for any
consequences resulting from such use.

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/

#include "Aria/Aria.h"
#include "ArNetworking/ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#include "ArGPSLocalizationTask.h"
//#include "ArGPSMapTools.h"
#include "ArSystemStatus.h"

#include <assert.h>

#include <string>
#include <map>

/** This class switches between laser and GPS localization tasks.
    This class lets you deactivate a localization task and activate another from
    some external command or trigger (e.g. user network command, at a specific goal,
    etc.)

    This is different from ArLocalizationManager.  ArLocalizationManager keeps
    multiple localization methods active at the same time, and combines their
    results, and therefore implements the ArBaseLocalizationTask interface.

    When switching to a new localization 
    it will block until the new localization task is not lost, or it times
    out (if a timeout was given).
*/
class LocSwitcher : public virtual ArASyncTask
{


public:
  LocSwitcher(ArRobot *robot, ArLocalizationTask *laserLoc, ArGPSLocalizationTask *gpsLoc,
    ArServerHandlerCommands *cmdSrv = 0/*, 
    ArActionLost *laserLostAct = 0, ArActionLost *gpsLostAct = 0, 
    ArActionSlowDownWhenNotCertain *laserSlowDownAct = 0,
ArActionSlowDownWhenNotCertain *gpsSlowDownAct = 0*/
);
  virtual ~LocSwitcher();
  bool switchToLaser(int timeoutSecs = -1);
  bool switchToGPS(int timeoutSecs = -1);
  void switchToLaserAsync();
  void switchToGPSAsync();

  ArBaseLocalizationTask* getActiveLoc()
  {
    return myActiveLoc;
  }

  std::string getActiveLocName()
  {
    return myActiveLoc->getShortName();
  }

private:
  const static int ourDefaultTimeout; ///< secs
  ArRobot *myRobot;
  ArLocalizationTask *myLaserLoc;
  ArGPSLocalizationTask *myGPSLoc;
  ArBaseLocalizationTask *myActiveLoc;
  ArServerHandlerCommands *myCommandServer;
  ArBaseLocalizationTask * myNextLoc;

  bool waitForLocalized(int timeoutSecs = -1);
  bool waitForLocIdle(int timeoutSecs = -1);

  virtual void *runThread(void *arg);
};



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


// used below
void reinitializeLocalization(ArRobot *robot, ArServerHandlerLocalization *locServer)
{
  locServer->localizeToPose(robot->getPose(), true);
}

// Re-initializes laser localization at the robot's current pose
class ReLocalizer
{
  ArRobot *myRobot;
  ArLocalizationTask *myLocTask;
  ArServerHandlerLocalization *myLocServer;
public:
  ReLocalizer(ArRobot *r, ArLocalizationTask *lt, ArServerHandlerLocalization
*ls) : myRobot(r), myLocTask(lt), myLocServer(ls)
  {}
  void reloc()
  {
    myRobot->lock();
    myRobot->stop();
    ArUtil::sleep(250);
    ArPose p = myRobot->getPose();
    myRobot->unlock();
    ArLog::log(ArLog::Normal, "ReLocalizer: reinitializing laser localization at (%.2f, %.2f, %.2f)", p.getX(), p.getY(), p.getTh());
    myLocTask->setRobotPose(p);
    //reinitializeLocalization(myRobot, myLocServer);
  }
};


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
      "Enable Motors", "Enabling Motors...",
      "Ignore", "Ignore"
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
#ifndef WIN32
  ArMTXIO mtxio;
#endif
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
#ifndef WIN32
  if(mtxio.isEnabled())
  {
    onCmd = &mtxOnCB;
    offCmd = &mtxOffCB;
  }
  else
#endif
  if(robot && (
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
#ifndef WIN32
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
#endif
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


  // Used to connect to a "central server" which can be used as a proxy
  // for multiple robot servers, and as a way for them to also communicate with
  // each other.  (objects implementing some of these inter-robot communication
  // features are created below).
  // NOTE: If the central server is running on the same host as robot server(s),
  // then you must use the -serverPort argument to instruct these robot-control
  // server(s) to use different ports than the default 7272, since the central
  // server will use that port.
  ArClientSwitchManager clientSwitch(&server, &parser);

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


  // Laser Localization
  ArLog::log(ArLog::Normal, "Creating laser localization task");
  ArLocalizationTask locTask(&robot, firstLaser, &map);





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

  /* Show the sample points used by MCL */
  ArDrawingData drawingDataL("polyDots", ArColor(0,255,0), 100, 75);
  ArFunctor2C<ArLocalizationTask, ArServerClient *, ArNetPacket *>
    drawingFunctorL(&locTask, &ArLocalizationTask::drawRangePoints);
  drawings.addDrawing(&drawingDataL, "Localization Points", &drawingFunctorL);

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

  // Enables the relocalization tool in MobileEyes:
  // In this example, this will only work with laser localisation (locTask),
  // not GPS.
  ArServerInfoLocalization serverInfoLocalization(&server, &robot, &locTask);
  ArServerHandlerLocalization serverLocHandler(&server, &robot, &locTask);


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

  // Wander mode
  ArServerModeWander modeWander(&server, &robot);


  // Add drive mode section to the configuration, and also some custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive.addControlCommands(&commands);

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



  // loc states
  Aria::getInfoGroup()->addStringString("Laser Loc Status", 10, new ArRetFunctorC<std::string, ArLocalizationTask>(&locTask, &ArLocalizationTask::getStateName));
  Aria::getInfoGroup()->addStringString("GPS Loc Status", 10, new ArRetFunctorC<std::string, ArGPSLocalizationTask>(&gpsLocTask, &ArGPSLocalizationTask::getStateName));
  

  // motor packet count, can diagnose slowdowns or communication problems
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10,
	  new ArConstRetFunctorC<int, ArRobot>(&robot,
					       &ArRobot::getMotorPacCount));


  // laser packet counts, can diagnose slowdowns or communication problems

  //std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot.getLaserMap()->begin();
       laserIt != robot.getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;
    if(!laser || !laser->isConnected()) continue;
    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10,
	    new ArRetFunctorC<int, ArLaser>(laser,
					 &ArLaser::getReadingCount));
  }


  Aria::getInfoGroup()->addStringDouble(
	  "Laser Loc Score", 8,
	  new ArRetFunctorC<double, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getLocalizationScore),
	  "%.03f");

  Aria::getInfoGroup()->addStringBool(
	  "Laser Loc Lost", 8,
	  new ArRetFunctorC<bool, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getRobotIsLostFlag));
  Aria::getInfoGroup()->addStringBool(
	  "Laser Loc Idle", 8,
	  new ArRetFunctorC<bool, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getIdleFlag));

  Aria::getInfoGroup()->addStringInt(
	  "Laser Loc Num Samples", 8,
	  new ArRetFunctorC<int, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getCurrentNumSamples),
	  "%4d");

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


  // Setup the dock if there is a docking system on board.
  ArServerModeDock *modeDock = NULL;
  modeDock = ArServerModeDock::createDock(&server, &robot, &locTask,
					  &pathTask, NULL, &parser);
  if (modeDock != NULL)
  {
    modeDock->checkDock();
    modeDock->addAsDefaultMode();
    modeDock->addToConfig(Aria::getConfig());
    modeDock->addControlCommands(&commands);
  }


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

  // Command that resets localization (for debugging but also sometimes useful
  // with MOGS).  See the reinitializeLocalization() above. This is the same as
  // using "Localize to Point" from MobileEyes at the exact localized pose of
  // the robot.
  commands.addCommand("ReinitLocalization", "Re-init localization task at current robot position", new ArGlobalFunctor2<ArRobot*, ArServerHandlerLocalization*>(&reinitializeLocalization, &robot, &serverLocHandler));

  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();
    // TODO move up near where stop mode is created?





  /* Services that allow the client to initiate scanning with the laser to
     create maps in Mapper3 (So not possible with SONARNL): */

  ArServerHandlerMapping handlerMapping(&server, &robot, firstLaser,
					fileDir, "", true);

  // make laser localization stop while mapping
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (&locTask, &ArLocalizationTask::setIdleFlag, true));

  // and then make it start again when we're doine
  handlerMapping.addMappingEndCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (&locTask, &ArLocalizationTask::setIdleFlag, false));

  // Save GPS positions in the .2d scan log when making a map
  handlerMapping.addLocationData("robotGPS",
			    gpsLocTask.getPoseInterpPositionCallback());

  // add the starting latitude and longitude info to the .2d scan log
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArGPSLocalizationTask, ArServerHandlerMapping *>
	  (&gpsLocTask, &ArGPSLocalizationTask::addScanInfo,
	   &handlerMapping));

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
    ArLog::log(ArLog::Normal, "   0) You can find this information in README.txt or docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   1) Connect to this server with MobileEyes");
    ArLog::log(ArLog::Normal, "   2) Go to Tools->Map Creation->Start Scan");
    ArLog::log(ArLog::Normal, "   3) Give the map a name and hit okay");
    ArLog::log(ArLog::Normal, "   4) Drive the robot around your space (see docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   5) Go to Tools->Map Creation->Stop Scan");
    ArLog::log(ArLog::Normal, "   6) Start up Mapper3");
    ArLog::log(ArLog::Normal, "   7) Go to File->Open on Robot");
    ArLog::log(ArLog::Normal, "   8) Select the .2d you created");
    ArLog::log(ArLog::Normal, "   9) Create a .map");
    ArLog::log(ArLog::Normal, "  10) Go to File->Save on Robot");
    ArLog::log(ArLog::Normal, "  11) In MobileEyes, go to Tools->Robot Config");
    ArLog::log(ArLog::Normal, "  12) Choose the Files section");
    ArLog::log(ArLog::Normal, "  13) Enter the path and name of your new .map file for the value of the Map parameter.");
    ArLog::log(ArLog::Normal, "  14) Press OK and your new map should become the map used");
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "\n   See docs/GPSMapping.txt for instructions on creating a map for GPS localization");
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal,
	     "Directory for maps and file serving: %s", fileDir);
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // Action to slow down robot when laser localization score drops but not lost.
  ArActionSlowDownWhenNotCertain actionSlowDown(&locTask, true);
  pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

  // Action to stop the robot when laser localization is "lost" (score too low)
  ArActionLost actionLostPath(&locTask, &pathTask, NULL, "Prevent navigation if laser localization is lost", true);
  pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

  // Prevent normal teleoperation driving if laser localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost actionLostRatioDrive(&locTask, &pathTask, &modeRatioDrive, "Prevent drive if laser localization lost", true /* do nothing if loc idle*/ );
  modeRatioDrive.getActionGroup()->addAction(&actionLostRatioDrive, 110);

  // also prevent wandering if laser lost:
  ArActionLost actionLostWander(&locTask, &pathTask, &modeWander, "Prevent wander if laser localization lost", true/*do nothing if loc idle*/ );
  modeWander.getActionGroup()->addAction(&actionLostWander, 110);



  // Action to slow down robot when gps localization score drops but not lost.
  ArActionSlowDownWhenNotCertain actionSlowDownGPS(&gpsLocTask, true);
  pathTask.getPathPlanActionGroup()->addAction(&actionSlowDownGPS, 140);

  // Action to stop the robot when gps localization is "lost" (score too low)
  ArActionLost actionGPSLostPath(&gpsLocTask, &pathTask, NULL, "Prevent navigation if GPS localization is lost", true);
  pathTask.getPathPlanActionGroup()->addAction(&actionGPSLostPath, 150);

  // Prevent normal teleoperation driving if gps localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost actionGPSLostRatioDrive(&gpsLocTask, &pathTask, &modeRatioDrive, "Prevent driving if GPS localization lost", true);
  modeRatioDrive.getActionGroup()->addAction(&actionGPSLostRatioDrive, 110);

  // Wander mode (also prevent wandering if gps lost):
  ArActionLost actionGPSLostWander(&gpsLocTask, &pathTask, &modeWander, "Prevent wander if GPS localization lost", true);
  modeWander.getActionGroup()->addAction(&actionGPSLostWander, 110);

  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping.addMappingStartCallback(actionLostPath.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostRatioDrive.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostWander.getDisableCB());
  handlerMapping.addMappingStartCallback(actionGPSLostPath.getDisableCB());
  handlerMapping.addMappingStartCallback(actionGPSLostRatioDrive.getDisableCB());
  handlerMapping.addMappingStartCallback(actionGPSLostWander.getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping.addMappingEndCallback(actionLostPath.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostRatioDrive.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostWander.getEnableCB());
  handlerMapping.addMappingEndCallback(actionGPSLostPath.getEnableCB());
  handlerMapping.addMappingEndCallback(actionGPSLostRatioDrive.getEnableCB());
  handlerMapping.addMappingEndCallback(actionGPSLostWander.getEnableCB());

  // Set up localization switcher. laser should start out active.
  gpsLocTask.setIdleFlag(true);
  LocSwitcher locSwitch(&robot, &locTask, &gpsLocTask, &commands);
  Aria::getInfoGroup()->addStringString("Current Localization", 10, new ArRetFunctorC<std::string, LocSwitcher>(&locSwitch, &LocSwitcher::getActiveLocName));

  // Do an initial localization of the robot. ARNL and SONARNL try all the home points
  // in the map, as well as the robot's current odometric position, as possible
  // places the robot is likely to be at startup.   If successful, it will
  // also save the position it found to be the best localized position as the
  // "Home" position, which can be obtained from the localization task (and is
  // used by the "Go to home" network request).
  // MOGS instead just initializes at the current GPS position.
  // (You will stil have to drive the robot so it can determine the robot's
  // heading, however. See GPS Mapping instructions.)

  locTask.localizeRobotAtHomeBlocking();
  gpsLocTask.localizeRobotInMapInit(robot.getPose(), 0.0, 0.0, 0.0, 0.1);


  // Let the client switch manager (for multirobot) spin off into its own thread
  // TODO move to multirobot example?
  clientSwitch.runAsync();

  // Start the networking server's thread
  server.runAsync();


  // Enable the motors and wait until the robot exits (disconnection, etc.) or this program is
  // canceled.
  robot.enableMotors();
  robot.waitForRunExit();
  Aria::exit(0);
}




const int LocSwitcher::ourDefaultTimeout = 20; // secs

LocSwitcher::LocSwitcher(ArRobot *robot, ArLocalizationTask *laserLoc, ArGPSLocalizationTask *gpsLoc, 
  ArServerHandlerCommands* cmdSrv /*,
    ArActionLost *laserLostAct, ArActionLost *gpsLostAct, 
    ArActionSlowDownWhenNotCertain *laserSlowDownAct, ArActionSlowDownWhenNotCertain *gpsSlowDownAct
*/
) : 
  myRobot(robot),
  myLaserLoc(laserLoc),
  myGPSLoc(gpsLoc),
  myActiveLoc(laserLoc),
  myCommandServer(cmdSrv)
  //,
  //myLaserLostAction(laserLostAct), myGPSLostAction(gpsLostAct), 
  //myLaserSlowDownAction(laserSlowDownAct), myGPSSlowDownAction(gpsSlowDownAct)
{
  if(cmdSrv)
  {
    cmdSrv->addCommand("Switch To Laser Localization", "Switch To Laser Localization", 
      new ArRetFunctor1C<bool, LocSwitcher, int>(this, &LocSwitcher::switchToLaser, -1));
    cmdSrv->addCommand("Switch To GPS Localization", "Switch To GPS Localization", 
      new ArRetFunctor1C<bool, LocSwitcher, int>(this, &LocSwitcher::switchToGPS, -1));
    // Note memory leak  from allocating functor objects, these are not deleted,
    // only clreate one LocSwither.
  }
}

LocSwitcher::~LocSwitcher()
{
  // TODO need to remove commands to avoid invalid pointers but // // ServerHandlerCommands doesn't have remCommand().
  if(myCommandServer)
  {
    //myCommandServer->remCommand("Switch To Laser Localization");
    //myCommandServer->remCommand("Switch To GPS Localization");
  }
  // TODO cancel any async threads
}

bool LocSwitcher::switchToLaser(int timeoutSecs)
{
  if(myActiveLoc == myLaserLoc) return true;
  if(myActiveLoc == 0) return false;
  assert(myActiveLoc == myGPSLoc);

  // stop robot
  myRobot->lock();
  myRobot->stop();
  myRobot->unlock();
  ArUtil::sleep(250);

  // deactivate GPS
  ArLog::log(ArLog::Normal, "LocSwitcher: Deactivating GPS...");
//  if(myGPSLostAction) myGPSLostAction->disable();
//  if(myGPSSlowDownAction) myGPSSlowDownAction->forceDisable(true);
  myGPSLoc->setLocalizationIdle(true);
  waitForLocIdle(timeoutSecs);

  // activate Laser
  ArLog::log(ArLog::Normal, "LocSwitcher: Activating Laser...");
  myLaserLoc->setLocalizationIdle(false);
//  if(myLaserLostAction) myLaserLostAction->enable();
//  if(myLaserSlowDownAction) myLaserSlowDownAction->forceDisable(false);
  myRobot->lock();
  ArPose p = myRobot->getPose();
  myRobot->unlock();
  ArLog::log(ArLog::Normal, "LocSwitcher: reinitializing laser localization at (%.2f, %.2f, %.2f)", p.getX(), p.getY(), p.getTh());
  myLaserLoc->setRobotPose(p);
  myActiveLoc = myLaserLoc;
  return waitForLocalized();
}


bool LocSwitcher::switchToGPS(int timeoutSecs)
{
  if(myActiveLoc == myGPSLoc) return true;
  if(myActiveLoc == 0) return false;
  assert(myActiveLoc == myLaserLoc);

  // stop robot
  myRobot->lock();
  myRobot->stop();
  myRobot->unlock();
  ArUtil::sleep(250);

  // deactivate Laser
  ArLog::log(ArLog::Normal, "LocSwitcher: Deactivating Laser...");
//  if(myLaserLostAction) myLaserLostAction->disable();
//  if(myLaserSlowDownAction) myLaserSlowDownAction->forceDisable(true);
  myLaserLoc->setLocalizationIdle(true);
  waitForLocIdle(timeoutSecs);

  // activate GPS
  ArLog::log(ArLog::Normal, "LocSwitcher: Activating GPS...");
  myGPSLoc->setLocalizationIdle(false);
//  if(myGPSLostAction) myGPSLostAction->enable();
//  if(myGPSSlowDownAction) myGPSSlowDownAction->forceDisable(false);
  myActiveLoc = myGPSLoc;
  return waitForLocalized();
}

bool LocSwitcher::waitForLocalized(int timeoutSecs)
{
  // todo check for thread cancellation
  if(timeoutSecs < 0) timeoutSecs = ourDefaultTimeout;
  ArTime start;
  ArBaseLocalizationTask *loc = myActiveLoc;
  std::string name = loc->getShortName();
  while(loc->getRobotIsLostFlag())
  {
    if(start.secSince() > timeoutSecs)
    {
      ArLog::log(ArLog::Normal, "LocSwitcher: Error: Timeout waiting for %s (%s) to localize (%d sec).", name.c_str(), loc->getShortName(), start.secSince());
      return false;
    }
    ArLog::log(ArLog::Normal, "LocSwitcher: Waiting for %s (%s) to localize...", name.c_str(), loc->getShortName());
    ArUtil::sleep(2000);
  }
  ArLog::log(ArLog::Normal, "LocSwitcher: %s is now localized", name.c_str());
  return true;
}

void LocSwitcher::switchToLaserAsync()
{
  // TODO cancel another thread if running and wait for it
  myNextLoc = myLaserLoc;
  runAsync();
}

void LocSwitcher::switchToGPSAsync()
{
  // TODO cancel another thread if running and wait for it
  myNextLoc = myGPSLoc;
  runAsync();
}

void *LocSwitcher::runThread(void*)
{
  // TODO check for thread cancellation
  if(myNextLoc == myLaserLoc)
    switchToLaser();
  else if(myNextLoc == myGPSLoc)
    switchToGPS();
  return 0;
}


bool LocSwitcher::waitForLocIdle(int timeoutSecs)
{
  // todo check for thread cancellation
  if(timeoutSecs < 0) timeoutSecs = 10;
  ArTime start;
  ArBaseLocalizationTask *loc = myActiveLoc;
  const std::string& name = loc->getShortName();
  while(!loc->getIdleFlag())
  {
    if(start.secSince() > timeoutSecs)
    {
      ArLog::log(ArLog::Normal, "LocSwitcher: Error: Timeout waiting for %s (%s) to become idle (%d sec).", name.c_str(), loc->getShortName(), start.secSince());
      return false;
    }
    ArLog::log(ArLog::Normal, "LocSwitcher: Waiting for %s (%s) to become idle...", name.c_str(), loc->getShortName());
    ArUtil::sleep(500);
  }
    ArLog::log(ArLog::Normal, "LocSwitcher: %s (%s) is now idle (inactive).", name.c_str(), loc->getShortName());
  return true;
}


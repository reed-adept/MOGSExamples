
/* User map-making tools for MOGS */

#ifndef ARGPSMAPTOOLS_H
#define ARGPSMAPTOOLS_H

#include "Aria.h"
#include "ArGPS.h"
#include "ArGPSCoords.h"
#include "ArNetworking.h"

/** Manages user "custom" commands that assist in map-making based on surveying
 * with the robot's GPS receiver.
 */
class GPSMapTools
{
  ArGPS *myGPS;
  ArMap *myMap;
  ArRobot *myRobot;
  ArServerHandlerPopup *myPopupServer;
  unsigned long myLastGoalNum;
  ArLLACoords myOrigin;
  ArMapGPSCoords myMapGPSCoords;
  bool myHaveMapGPSCoords;
  ArPose myForbiddenLineStart;
  bool myForbiddenLineStarted;
  ArPose myObstacleLineStart;
  bool myObstacleLineStarted;
  ArFunctor1C<GPSMapTools, ArArgumentBuilder*> myStartNewMapCB;
  ArFunctor1C<GPSMapTools, ArArgumentBuilder*> myAddGoalHereCB;
  ArFunctorC<GPSMapTools> mySetOriginCB;
  ArFunctorC<GPSMapTools> myStartForbiddenLineCB;
  ArFunctorC<GPSMapTools> myEndForbiddenLineCB;
  ArFunctorC<GPSMapTools> myStartObstacleLineCB;
  ArFunctorC<GPSMapTools> myEndObstacleLineCB;
  ArFunctorC<GPSMapTools> myReloadMapFileCB;
  ArServerHandlerMap *myMapServer;

  ArPose getCurrentPosFromGPS();
  bool checkGPS(const std::string &action);
  bool checkMap(const std::string &action);
  void startNewMap(ArArgumentBuilder *args);
  void addGoalHere(ArArgumentBuilder *args);
  void setOrigin();
  void startForbiddenLine();
  void endForbiddenLine();
  void startObstacleLine();
  void endObstacleLine();
  void reloadMapFile();
  void resetMapCoords(const ArLLACoords& mapOrigin);
public:
  /** Adds custom commands to @a commandServer to drop goals at GPS position, ... 
   */
  AREXPORT GPSMapTools(ArGPS *gps, ArRobot *robot, ArServerHandlerCommands *commandServer, ArMap *map, ArServerHandlerMap *mapserver, ArServerHandlerPopup *popupServer = NULL);
};

#endif

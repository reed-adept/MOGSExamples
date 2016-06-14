/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/

#ifndef ACTIONGOTOSTRAIGHT_H
#define ACTIONGOTOSTRAIGHT_H

// Modified version of ARIA's ArActionGotoStraight; changes may be
// incorporated into ArActionGotoStraight at some point.

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/** This action sets a heading and forward velocity based on angle and distance
    from current robot position to a given goal point.

   This action naively drives straight towards a given ArPose. The
   action stops the robot when it is within a distance threshold (Close Dist.)
   to the goal point, or alternatively if it has gone the expected distance 
   to the goal point.  Maximum forward velocity can be set in the constructor
   or by calling setSpeed().  The velocity requested is proportional to the 
   remaining distance to the goal, but is limited to the speed set with
   setSpeed().  The threshold for considering the goal achieved may be 
   set by calling setCloseDist().

   You can give it a new goal pose with setGoal(), cancel its movement
   with cancelGoal(), and see if it got there with haveAchievedGoal().
   You can reverse the motion by setting the backwards flag to true when
   setting a new goal.  If you set the justDistance flag to true, the action
   stops after covering the calculated distance to the goal rather than 
   checking each iteration against the given Close Distance threshold.

   Normally a new heading and forward velocity will be requested simultaneously.
   To make the robot rotate in place before forward velocity, set the
   turn threshold by calling setTurnThreshold().  This may be desired for
   goals that are only a short distance away from the robot, to avoid a large
   arc of motion.

   This action does not check sensors.  You may use a higher priority action to
   stop or otherwise override motion based on sensor data (e.g.
   ArActionLimiterForward, or you own custom action.)

   @ingroup ActionClasses
**/


class ActionGotoStraight : public ArAction
{
public:
  AREXPORT ActionGotoStraight(const char *name = "goto", 
				double speed = 400);
  AREXPORT virtual ~ActionGotoStraight();

  /// Sees if the goal has been achieved
  AREXPORT bool haveAchievedGoal(void);
  /// Cancels the goal the robot has
  AREXPORT void cancelGoal(void);
  /// Sets a new goal and sets the action to go there
  AREXPORT void setGoal(ArPose goal, bool backwards = false, 
			bool justDistance = true);
  /// Sets the goal in a relative way
  AREXPORT void setGoalRel(double dist, double deltaHeading, 
			   bool backwards = false, bool justDistance = true);
  /// Gets the goal the action has
  ArPose getGoal(void) { return myGoal; }
  /// Gets whether we're using the encoder goal or the normal goal
  bool usingEncoderGoal(void) { return myUseEncoderGoal; }
  /// Sets a new goal and sets the action to go there
  AREXPORT void setEncoderGoal(ArPose encoderGoal, bool backwards = false,
			       bool justDistance = true);
  /// Sets the goal in a relative way
  AREXPORT void setEncoderGoalRel(double dist, double deltaHeading, 
				  bool backwards = false, 
				  bool justDistance = true);
  /// Gets the goal the action has
  ArPose getEncoderGoal(void) { return myEncoderGoal; }
  /// Sets the speed the action will travel to the goal at (mm/sec)
  void setSpeed(double speed) { mySpeed = speed; }
  /// Gets the speed the action will travel to the goal at (mm/sec)
  double getSpeed(void) { return mySpeed; }
  /// Sets how close we have to get if we're not in just distance mode
  void setCloseDist(double closeDist = 100) { myCloseDist = closeDist; } 
  /// Gets how close we have to get if we're not in just distance mode
  double getCloseDist(void) { return myCloseDist; }
  /// Sets whether we're backing up there or not (set in the setGoals)
  bool getBacking(void) { return myBacking; }
  
  /// @warning if threshAngle is too small (smaller than about 20 deg) then
  /// the robot may not achieve that heading, and it could become stuck with no
  /// movement.
  void setTurnThreshold(double threshAngle) { myTurnThresh = threshAngle; }

protected:
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif

  ArPose myGoal;
  bool myUseEncoderGoal;
  ArPose myEncoderGoal;
  double mySpeed;
  bool myBacking;
  ArActionDesired myDesired;
  const bool myPrinting;
  double myDist;
  double myCloseDist;
  bool myJustDist;
  double myDistTravelled;
  ArPose myLastPose;
  double myTurnThresh;
  
  enum State
  {
    STATE_NO_GOAL, 
    STATE_ACHIEVED_GOAL,
    STATE_GOING_TO_GOAL
  };
  State myState;
};

#endif // ACTIONGOTO

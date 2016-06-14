#ifndef ACTIONGOTOSTRAIGHT_H
#define ACTIONGOTOSTRAIGHT_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

/// This action goes to a given ArPose very naively

/**
   This action naively drives straight towards a given ArPose. The
   action stops the robot when it has travelled the distance that that
   pose is away. It travels at 'speed' mm/sec.

   You can give it a new goal pose with setGoal(), cancel its movement
   with cancelGoal(), and see if it got there with haveAchievedGoal().

   For arguments to the goals and encoder goals you can tell it to go
   backwards by calling them with the backwards parameter true.  If
   you set the justDistance to true it will only really care about
   having driven the distance, if false it'll try to get to the spot
   you wanted within close distance.

   This doesn't avoid obstacles or anything, you could add have an obstacle
   avoidance ArAction at a higher priority to try to do this. (For
   truly intelligent navigation, see the ARNL and SONARNL software libraries.)
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

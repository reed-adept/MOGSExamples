#include "ArExport.h"
#include "ariaOSDef.h"
#include "ActionGotoStraight.h"
#include "ArRobot.h"

#define DEBUG false // if true enables misc debug logging

AREXPORT ActionGotoStraight::ActionGotoStraight(const char *name,
						    double speed) :
  ArAction(name, "Goes to the given goal."),
  myPrinting(DEBUG)
{
  setNextArgument(ArArg("speed", &mySpeed, 
			"Speed to travel to goal at. (mm/sec)"));
  mySpeed = speed;
  myState = STATE_NO_GOAL;

  myUseEncoderGoal = false;
  myBacking = false;
  setCloseDist();
  myTurnThresh = 360;
}

AREXPORT ActionGotoStraight::~ActionGotoStraight()
{

}

AREXPORT bool ActionGotoStraight::haveAchievedGoal(void)
{
  if (myState == STATE_ACHIEVED_GOAL)
    return true;
  else
    return false;
}

AREXPORT void ActionGotoStraight::cancelGoal(void)
{
  myState = STATE_NO_GOAL;
}

AREXPORT void ActionGotoStraight::setGoal(ArPose goal, bool backToGoal, 
					    bool justDistance)
{
  myState = STATE_GOING_TO_GOAL;
  myGoal = goal;
  myUseEncoderGoal = false;
  myBacking = backToGoal;
  myLastPose = myRobot->getPose();
  myDist = myRobot->getPose().findDistanceTo(goal);
  myJustDist = true;
  myDistTravelled = 0;
}

AREXPORT void ActionGotoStraight::setGoalRel(double dist, 
					       double deltaHeading,
					       bool backToGoal, 
					       bool justDistance)
{
  ArPose goal;
  goal.setX(dist * ArMath::cos(deltaHeading));
  goal.setY(dist * ArMath::sin(deltaHeading));
  goal = myRobot->getToGlobalTransform().doTransform(goal);
  setGoal(goal, backToGoal, justDistance);
}

AREXPORT void ActionGotoStraight::setEncoderGoal(ArPose encoderGoal, 
						   bool backToGoal,
						   bool justDistance)
{
  myState = STATE_GOING_TO_GOAL;
  myEncoderGoal = encoderGoal;
  myUseEncoderGoal = true;
  myBacking = backToGoal;
  myDist = myRobot->getEncoderPose().findDistanceTo(encoderGoal);
  myJustDist = justDistance;
  myDistTravelled = 0;
  myLastPose = myRobot->getEncoderPose();
}

AREXPORT void ActionGotoStraight::setEncoderGoalRel(double dist, 
						      double deltaHeading,
						      bool backToGoal,
						      bool justDistance)
{
  ArPose goal;
  goal.setX(dist * ArMath::cos(deltaHeading));
  goal.setY(dist * ArMath::sin(deltaHeading));
  goal = myRobot->getToGlobalTransform().doTransform(goal);
  goal = myRobot->getEncoderTransform().doInvTransform(goal);
  setEncoderGoal(goal, backToGoal, justDistance);
}

AREXPORT ArActionDesired *ActionGotoStraight::fire(ArActionDesired currentDesired)
{
  double angle;
  double dist;
  double distToGo;
  double vel;

  // if we're there we don't do anything
  if (myState == STATE_ACHIEVED_GOAL || myState == STATE_NO_GOAL)
    return NULL;


  ArPose goal;
  if (!myUseEncoderGoal)
  {
    goal = myGoal;
    myDistTravelled += myRobot->getPose().findDistanceTo(myLastPose);
    myLastPose = myRobot->getPose();
  }
  else
  {
    goal = myRobot->getEncoderTransform().doTransform(myEncoderGoal);
    myDistTravelled += myRobot->getEncoderPose().findDistanceTo(myLastPose);
    myLastPose = myRobot->getEncoderPose();
  }

  if (myJustDist)
  {
    distToGo = myDist - myDistTravelled;
    dist = fabs(distToGo);
  }
  else
  {
    dist = myRobot->getPose().findDistanceTo(goal);
  }

  if (((myJustDist && distToGo <= 0) || 
       (!myJustDist && dist < myCloseDist))
      && ArMath::fabs(myRobot->getVel() < 5))
  {
    if (myPrinting)
      ArLog::log(ArLog::Normal, "Achieved goal");
    myState = STATE_ACHIEVED_GOAL;
    myDesired.setVel(0);
    myDesired.setDeltaHeading(0);
    return &myDesired;  
  }

  // see where we want to point
  angle = myRobot->getPose().findAngleTo(goal);
  if (myBacking)
    angle = ArMath::subAngle(angle, 180);
  myDesired.setHeading(angle);
  const double angleDelta = ArMath::subAngle(myLastPose.getTh(), angle);

  // drive forward (or back if backing up flag).  if we're close, 
  // stop. if the angle turned above is bigger than turnThresh, i
  // don't start driving forward yet.
  if ((myJustDist && distToGo <= 0) || 
      (!myJustDist && dist < myCloseDist))
  {
    myDesired.setVel(0);
    vel = 0;
  }
  else if(fabs(angleDelta) <= myTurnThresh)
  {
    vel = sqrt(dist * 200 * 2);
    if (vel > mySpeed)
      vel = mySpeed;
    if (myBacking)
      vel *= -1;
    myDesired.setVel(vel);
  }
  if (myPrinting)
    ArLog::log(ArLog::Normal, "dist %.0f angle %.0f angleDelta %.0f vel %.0f turnThresh %.0f desiredHeading %.0f desiredVel %.0f", 
	       dist, angle, angleDelta, vel, myTurnThresh, myDesired.getHeading(), myDesired.getVel());
  return &myDesired;
}


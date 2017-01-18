
#ifndef RECENTER_WHEELS_H
#define RECENTER_WHEELS_H

/* At each goal, check the name of the goal.  If it starts with the 
   given prefix, then recenter the Seekur wheels. 
*/


class RecenterWheels
{
private:
  ArPathPlanningInterface *myPathTask;
  ArRobot *myRobot;
  std::string myGoalNamePrefix;
  ArFunctor1C<RecenterWheels, ArPose> myGoalCB;

public:
  RecenterWheels(ArPathPlanningInterface *pathTask, ArRobot *robot, const std::string& goalNamePrefix) : 
    myPathTask(pathTask),
    myRobot(robot), 
    myGoalNamePrefix(goalNamePrefix), 
    myGoalCB(this, &RecenterWheels::goalReached)
  {
    pathTask->addGoalDoneCB(&myGoalCB);
    ArLog::log(ArLog::Normal, "RecenterWheels: will check goals with name beginning \"%s\"", goalNamePrefix.c_str());
  }

  ~RecenterWheels()
  {
    myPathTask->remGoalDoneCB(&myGoalCB);
  }

private:
  void goalReached(ArPose pose)
  {
    const std::string goalname = myPathTask->getCurrentGoalName();
    if(goalname.compare(0, myGoalNamePrefix.size(), myGoalNamePrefix) == 0)
    {
      ArLog::log(ArLog::Normal, "Recentering Seekur Wheels...");
      myRobot->com(120);
    }
  }
};
  


#endif

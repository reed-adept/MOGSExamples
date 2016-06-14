#ifndef ACTIONSPEEDLIMITER_H
#define ACTIONSPEEDLIMITER_H

// Modified version of ARIA's ActionLimiterForwards; changes may be
// incorporated into ActionLimiterForwards at some point.

#include "ariaTypedefs.h"
#include "ArAction.h"

class ArRangeDevice;

/// Action to limit the forwards motion of the robot based on range sensor readings.
/**
   This action uses the sensors to find a maximum forwared speed to travel at; when the range
   sensor (e.g. sonar or laser) detects obstacles closer than the given parameters,
   this action requests that the robot decelerate or stop.
   @ingroup ActionClasses
*/
class ActionLimiterForwards : public ArAction
{
public:
  /// Constructor
  AREXPORT ActionLimiterForwards(const char *name = "speed limiter", 
				   double stopDistance = 250,
				   double slowDistance = 1000,
				   double slowSpeed = 200,
				   double widthRatio = 1);
  /// Destructor
  AREXPORT virtual ~ActionLimiterForwards();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT void setParameters(double stopDistance = 250,
			      double slowDistance = 1000,
			      double slowSpeed = 200,
			      double widthRatio = 1);

  bool getStopped() const { return myLastStopped; } 
  ArPose getLastSensorReadingPos() const { return myLastSensorReadingPos; } 
  const ArRangeDevice* getLastSensorReadingDevice() const { return myLastSensorReadingDev; } 
protected:
  bool myLastStopped;
  double myStopDist;
  double mySlowDist;
  double mySlowSpeed;
  double myWidthRatio;
  ArActionDesired myDesired;
  ArPose myLastSensorReadingPos;
  const ArRangeDevice *myLastSensorReadingDev;
};

#endif // ARACTIONSPEEDLIMITER_H

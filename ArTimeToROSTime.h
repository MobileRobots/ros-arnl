
#ifndef ARTIMETOROSTIME_H
#define ARTIMETOROSTIME_H


#include <ros/ros.h>
#include "ariaUtil.h"

inline ros::Time convertArTimeToROS(const ArTime& t)
{
  // ARIA/ARNL times are in reference to an arbitrary starting time, not OS
  // clock, so find the time elapsed between now and t
  // to adjust the time stamp in ROS time vs. now accordingly.
  ArTime arianow;
  const double dtsec = (double) t.mSecSince(arianow) / 1000.0;
#ifdef DEBUG_ARTIMETOROSTIME
  printf("convertArTimeToROS: ArTime was %f seconds ago\n", dtsec);
#endif
  return ros::Time(ros::Time::now().toSec() - dtsec);
}

inline ArTime convertROSTimeToArTime(const ros::Time& t)
{
  // XXX TODO FIX
  ArTime arnow;
  arnow.addMSec(-1000.0 * (t.toSec() - ros::Time::now().toSec())); // convert from sec to msec and subtract
  return arnow;
}


#endif

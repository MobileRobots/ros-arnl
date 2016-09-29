#ifndef ROSARNLLASERSUBSCRIBER_H
#define ROSARNLLASERSUBSCRIBER_H

#include "Aria.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>



class ROSLaserScanRangeDevice : public virtual ArRangeDevice
{
  ros::Subscriber sub;
  unsigned int myCounter;
  std::vector<ArSensorReading> myReadings;
public:
  ROSLaserScanRangeDevice(const char *scantopic, ros::NodeHandle& nh, const char *name = NULL);
  virtual ~ROSLaserScanRangeDevice();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);

// ArRobot::addRangeDevice() calls ArRangeDevice::setRobot(), override to
// get pointer to ArRobot object and add our task.  ArRangeDevice() also
/* stores a pointer to the ArRobot object which can be obtained from
   / ArRangeDevice::getRobot().
   virtual void setRobot(ArRobot *robot)
   {
   if(robot)
   robot->addSensorInterpTask("ROSLaserScanRangeDevice", 20, &myUpdateTask);
   ArRangeDevice::setRobot(robot);
   }
*/

};

#endif


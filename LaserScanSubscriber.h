#ifndef ROSARNLLASERSUBSCRIBER_H
#define ROSARNLLASERSUBSCRIBER_H

#include "Aria.h"
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>



/**
  @note currently only sensor translation relative to base_link is used, not any
        angle. this should be added in the future.
*/
class ROSLaserScanRangeDevice : public virtual ArRangeDevice
{
  ros::Subscriber sub;
  unsigned int myCounter;
  std::vector<ArSensorReading> myReadings;
  tf::TransformListener tfListener;
  std::string tfname;
public:
  ROSLaserScanRangeDevice(const std::string& scantopic, ros::NodeHandle& nh, const std::string& sensor_tf_name,  const std::string& name = "");
  virtual ~ROSLaserScanRangeDevice();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif


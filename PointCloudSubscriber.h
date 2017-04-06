#ifndef ROSARNLPOINTCLOUDSUBSCRIBER_H
#define ROSARNLPOINTCLOUDSUBSCRIBER_H

#include "Aria.h"

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>


/** Subscribe to ROS PointCloud2 topic and import data into ARIA or ARNL as a range sensor. */
class ROSPointCloudRangeDevice : public virtual ArRangeDevice
{
  ros::Subscriber sub;
  unsigned int myCounter;
  std::vector<ArSensorReading> myReadings;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::string sensor_tfname; 
  std::string global_tfname;
  int min_z; ///< mm
  int max_z; ///< mm
  bool limit_min_z;
  bool limit_max_z;
public:
  ROSPointCloudRangeDevice(const std::string& topic, ros::NodeHandle& nh, const std::string& sensor_tf_name,  const std::string& global_tf_name = "map", const std::string& name = "");
  virtual ~ROSPointCloudRangeDevice();
  void processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& pc);
  /// milimeters
  void filterZBelow(int zmax) { max_z = zmax; limit_max_z = true; }
  /// milimeters 
  void filterZAbove(int zmin) { min_z = zmin; limit_min_z = true; }
};

#endif


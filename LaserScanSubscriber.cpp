
#include "Aria.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "LaserScanSubscriber.h"
#include "ArTimeToROSTime.h"


/* TODO
      * Use message timestamp and other timing information
      * Use a tf for the sensor rather than assuming its at 0,0 on the robot
*/       

ROSLaserScanRangeDevice::ROSLaserScanRangeDevice(const std::string& scantopic,
ros::NodeHandle& nh, const std::string& _tfname, const std::string& name) : ArRangeDevice(
    300,  // Current buffer size. Choose a value big enough for at least one set of readings
    500,  // Cumulative buffer size
    (name == "")?scantopic.c_str():name.c_str(), // ARIA sensor Name
    25000 // Default maximum range limit. Will be reset later. Must be >= sensor's maximum range
  ),
    myCounter(0),
    tfname(_tfname)
{
 

  // configure ArNetworking (MobileEyes) visualization properties:
  setCurrentDrawingData(new ArDrawingData("polyDots", ArColor(200, 50, 0), 100, 75), true); 
  setCumulativeDrawingData(new ArDrawingData("polyDots", ArColor(100, 20, 0), 120, 74), true); 

  // Subscribe to ROS topic
  sub = nh.subscribe<sensor_msgs::LaserScan>(scantopic, 10, &ROSLaserScanRangeDevice::processLaserScan, this);

  ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Subscribed to laser scan data from \"%s\" topic, tf frame \"%s\"", scantopic.c_str(), tfname.c_str());
}

ROSLaserScanRangeDevice::~ROSLaserScanRangeDevice()
{
  //if(ArRobot *r = getRobot())
  //  r->remSensorInterpTask("ROSLaserScanRangeDevice");

  // todo do we need to unsubscribe or will sub do it when destroyed?
}

void ROSLaserScanRangeDevice::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  /*scan->ranges is a std::vector of floats
  for(std::vector<float>::const_iterator i = scan->ranges.begin(); i != scan->ranges.end(); ++i)
    {
      float r = (*i);
if(std::isfinite(r))
  ArLog::log(ArLog::Normal, "ROSLaserScanRangeDevice: range is %f meters\n", r);
    }
  */


  lockDevice();

  myReadings.resize(scan->ranges.size());
  setMaxRange(scan->range_max);
  float angle = scan->angle_min;
  ros::Time t(scan->header.stamp);
  const ros::Duration scanduration(scan->scan_time);
  for(size_t i = 0; i < scan->ranges.size(); ++i)
  {
    const float range = scan->ranges[i];
    if(std::isfinite(range))
    {
      tf::StampedTransform transform;
      tfListener.lookupTransform(tfname, "base_link", t, transform);
      const float x = transform.getOrigin().x();
      const float y = transform.getOrigin().y();
      // todo adjust angle based on transform
      // todo take other rotation axes (and z) into account
      myReadings[i].resetSensorPosition(x, y, ArMath::radToDeg(angle)); 
      ArRobot *robot = getRobot();

      // todo interpolate robot pose based on sensor tf timestamp
      const ArPose robotPose = robot->getPose();
      const ArPose robotEncoderPose = robot->getEncoderPose();
      const ArTransform toGlobal = robot->getToGlobalTransform();       //get robot position, encoder position, transform, counter, etc. 
                                                                         //here same as in the ExampleRangeDevice

      myReadings[i].newData( (range*1000.0), robotPose, robotEncoderPose, toGlobal, myCounter++, convertROSTimeToArTime(t));
      myCurrentBuffer.addReading(myReadings[i].getX(), myReadings[i].getY());
    }
    angle += scan->angle_increment;
    t += scanduration;
  }

  unlockDevice();
}
  


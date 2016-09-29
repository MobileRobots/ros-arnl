
#include "Aria.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "LaserScanSubscriber.h"


/* TODO
      * Use message timestamp and other timing information
      * Use a tf for the sensor rather than assuming its at 0,0 on the robot
*/       

ROSLaserScanRangeDevice::ROSLaserScanRangeDevice(const char *scantopic, ros::NodeHandle& nh, const char *name) : ArRangeDevice(
    300,  // Current buffer size. Choose a value big enough for at least one set of readings
    500,  // Cumulative buffer size
    name?name:scantopic, // Name
    25000 // Default maximum range limit. Will be reset later. Must be >= sensor's maximum range
  ),
    myCounter(0)
{
 

  // configure ArNetworking (MobileEyes) visualization properties:
  setCurrentDrawingData(new ArDrawingData("polyDots", ArColor(200, 50, 0), 100, 75), true); 
  setCumulativeDrawingData(new ArDrawingData("polyDots", ArColor(100, 20, 0), 120, 74), true); 

  // Subscribe to ROS topic
  sub = nh.subscribe<sensor_msgs::LaserScan>(scantopic, 10, &ROSLaserScanRangeDevice::processLaserScan, this);

  ROS_INFO_NAMED("rosarnl_node", "rosarnl_node: Subscribed to laser scan data from \"%s\" topic.", scantopic);
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

  // TODO we need the tf information from the device and add that transform, or
  // use that in resetSensorPosition().

  myReadings.resize(scan->ranges.size());
  setMaxRange(scan->range_max);
  float angle = scan->angle_min;
  for(size_t i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
    if(std::isfinite(range))
    {
      myReadings[i].resetSensorPosition(0, 0, ArMath::radToDeg(angle)); // todo someday set correct position of reading using tf instead of 0, 0
      ArRobot *robot = getRobot();
      ArPose robotPose = robot->getPose();
      ArPose robotEncoderPose = robot->getEncoderPose();
      ArTime currentTime; // will be set to current system time. TODO use timestamp from header plus i * time_increment
      ArTransform toGlobal = robot->getToGlobalTransform();              //get robot position, encoder position, transform, counter, etc. 
                                                                         //here same as in the ExampleRangeDevice
      myReadings[i].newData( (range*1000.0), robotPose, robotEncoderPose, toGlobal, myCounter++, currentTime);
      myCurrentBuffer.addReading(myReadings[i].getX(), myReadings[i].getY());
    }
    angle += scan->angle_increment;
  }

  unlockDevice();
}
  


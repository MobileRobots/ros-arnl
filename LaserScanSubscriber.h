#ifndef ROSARNLLASERSUBSCRIBER_H
#define ROSARNLLASERSUBSCRIBER_H

#include "Aria.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


// TODO do the topic subscription in this class

class ROSLaserScanRangeDevice : public virtual ArRangeDevice
{
  ros::Subscriber sub;
  unsigned int myCounter;
  std::vector<ArSensorReading> myReadings;
public:
  ROSLaserScanRangeDEvice(const char *scantopic, ros::NodeHandle& nh, const char *name = NULL) : ArRangeDevice(
    300,  // Current buffer size. Choose a value big enough for at least one set of readings
    500,  // Cumulative buffer size
    name?name:scantopic, // Name
    25000 // Default maximum range limit. Will be reset later. Must be >= sensor's maximum range
  ),
    myCounter(0)
  {
   
    sub = nh.subscribe<sensor_msgs::LaserScan>(scantopic, 10, &processLaserScan, this);

    // TODO get laserscan topic format data and set up myReadings, max range,

    // etc. based on that, either here or when we get
    // the first message or check when receiving messages if its changed,
    // instead of the following:
    // this simulates something like a scanning laser rangefinder in which all
    // range measurements are at different angles relative to the same origin
    // point.   we will simulate a sensor placed on the robot 250 mm in front
    // of the robot's center point, and we will generate 5 fake readings: every
    // 9 degrees over a +/-45 degree range.
    for(float angle = -45; angle <= 45; angle += 10)
    {
      myReadings.push_back(ArSensorReading(250, 0, angle));
    }

    ArLog::log(ArLog::Normal, "ROSLaserScanRangeDevice: Will provide a set of %d readings each update", myReadings.size());

    // configure visualization properties:
    setCurrentDrawingData(new ArDrawingData("polyDots", ArColor(200, 50, 0), 100, 75), true); 
    setCumulativeDrawingData(new ArDrawingData("polyDots", ArColor(100, 20, 0), 120, 74), true); 
  }

  virtual ~ROSLaserScanRangeDevice()
  {
    //if(ArRobot *r = getRobot())
    //  r->remSensorInterpTask("ROSLaserScanRangeDevice");

    // todo do we need to unsubscribe or will sub do it when destroyed?
  }

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

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
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

    // TODO we need the tf information from the device and add that transform

    myReadings.resize(scan->ranges.size());
    float angle = scan->angle_min;
    for(size_t i = 0; i < scan->ranges.size(); ++i)
    {
      float range = scan->ranges[i];
      if(std::isfinite(range))
      {
        myReadings[i].resetSensorPosition(0, 0, ArMath::radToDeg(angle)); // todo someday set correct position of reading instead of 0, 0
        ArRobot *robot = getRobot();
        ArPose robotPose = robot->getPose();
        ArPose robotEncoderPose = robot->getEncoderPose();
        ArTime currentTime; // will be set to current system time. 
        ArTransform toGlobal = robot->getToGlobalTransform();              //get robot position, encoder position, transform, counter, etc. 
                                                                           //here same as in the ExampleRangeDevice
        myReadings[i].newData( (range*1000.0), robotPose, robotEncoderPose, toGlobal, myCounter++, currentTime);
        myCurrentBuffer.addReading(myReadings[i].getX(), myReadings[i].getY());
      }
      angle += scan->angle_increment;
    }

    unlockDevice();
  }
    
};

#endif


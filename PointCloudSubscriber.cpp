
#include "Aria.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include "PointCloudSubscriber.h"
#include "ArTimeToROSTime.h"
#include "ROSNamesUtil.h"


/* TODO
      * Use message timestamp and other timing information
      * Use a tf for the sensor rather than assuming its at 0,0 on the robot
*/       

ROSPointCloudRangeDevice::ROSPointCloudRangeDevice(const std::string& topic,
  ros::NodeHandle& nh, const std::string& _tfname, const std::string& _global_tfname, 
  const std::string& devname) : ArRangeDevice(
    300,  // Current buffer size. Choose a value big enough for at least one set of readings
    500,  // Cumulative buffer size
    (devname == "")?withoutLeadingSlash(topic).c_str():devname.c_str(), // ARIA sensor Name
    25000 // Default maximum range limit. Will be reset later. Must be >= sensor's maximum range
  ),
    myCounter(0),
    tfListener(tfBuffer),
    sensor_tfname(_tfname),
    global_tfname(_global_tfname),
    limit_min_z(false),
    limit_max_z(false)
{
  // configure ArNetworking (MobileEyes) visualization properties:
  // todo select different colors for different devices
  // ArDrawingData args are type, color, size, layer
  setCurrentDrawingData(new ArDrawingData("polyDots", ArColor(200, 50, 0), 40, 75), true); 
  setCumulativeDrawingData(new ArDrawingData("polyDots", ArColor(100, 20, 0), 60, 74), true); 

  // todo wait for tf?

  // Subscribe to ROS topic
  sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 10, &ROSPointCloudRangeDevice::processPointCloud2, this);

  ROS_INFO_NAMED("rosarnl_node::ROSPointCloudRangeDevice", "rosarnl_node: Subscribed to sensor point cloud data from \"%s\" topic, sensor tf frame = \"%s\", global tf frame (should match ARNL map frame) = \"%s\"", topic.c_str(), sensor_tfname.c_str(), global_tfname.c_str());

}

ROSPointCloudRangeDevice::~ROSPointCloudRangeDevice()
{
  // todo do we need to unsubscribe or will sub do it when destroyed?
}

/// ROS topic subscriber callback, called for each new PointCloud2 messsage 
void ROSPointCloudRangeDevice::processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  lockDevice();

  ros::Time t(pc->header.stamp);
  // todo interpolate robot pose based on timestamp
  
  sensor_msgs::PointCloud2 pct;

  try {
    geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(sensor_tfname, global_tfname, t, ros::Duration(0.080));
    // todo try map frame, if odom or map don't exist, use base_link then transform using aria below
    // todo catch exceptions
    tf2::doTransform(*pc, pct, transform);
  } catch (tf2::TransformException &e) {
    ROS_WARN_NAMED("rosarnl_node::ROSPointCloudRangeDevice", "rosarnl_node: Error transforming pointcloud data from sensor frame \"%s\" to global frame \"%s\": %s. Skipping this message.", sensor_tfname.c_str(), global_tfname.c_str(), e.what());
    return;
  }

  sensor_msgs::PointCloud2Iterator<float> ix(pct, "x"), iy(pct, "y"), iz(pct, "z");

  for(; ix != ix.end() && iy != iy.end() && iz != iz.end(); ++ix, ++iy, ++iz)
  {
    //ArRobot *robot = getRobot();
    //const ArPose robotPose = robot->getPose();
    //const ArPose robotEncoderPose = robot->getEncoderPose();
    //const ArTransform toGlobal = robot->getToGlobalTransform();       //get robot position, encoder position, transform, counter, etc. 

    //myReadings[i].newData( (range*1000.0), robotPose, robotEncoderPose, toGlobal, myCounter++, convertROSTimeToArTime(t));

    // TODO transform x, y to global?

    // convert from m to mm
    const float x = (*ix) * 1000.0;
    const float y = (*iy) * 1000.0;
    const float z = (*iz) * 1000.0;

    // optionally filter out high and low points, add to buffer
    // todo include a band below a floor to sort of represent negative obstacles
    if( (!limit_min_z || z > min_z) &&
        (!limit_max_z || z < max_z) )
    {
      myCurrentBuffer.addReading(x, y);
    }
    //else printf("filtered(%.0f, %.0f, %.0f)\n", x, y, z);
      
  }

  unlockDevice();
}
  


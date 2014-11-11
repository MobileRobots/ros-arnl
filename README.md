ros-arnl
========

Provides ROS interface to basic ARNL features. Requires ARNL and ArnlBase to be installed separately.

The rosarnl node provides a subset of the standard ROS navigation topic
interface:
 * goal       A goal pose to begin navigating to
 * goalname   Name of a goal in ARNL's map to begin navigating to.
 * cancel     Cancel navigation and switch to idle stopped state.
 * feedback
 * status
 * amcl_pose  Localized pose of robot
 * init       An initial pose to manually localize to
 * enable_motors  Enable/disable the robot motors.
 * laser_scan Laser data

Transform (tf) data are provided for the robot base in the map
 and the laser scan.
 
In addition, ARNL ArNetworking services are provided, so ARNL can be accessed
and configured via MobileEyes or other ArNetworking clients. Use MobileEyes
to perform scanning for map generation, and use Mapper3 to process the scan
and upload the map to ARNL.

This is intended for simple operation of ARNL navigation, and as a base for
further features to be added.

See LICENSE.txt for the license terms for this code.  ARNL and ArnlBase are
provided under their own MobileRobots Individual Software License.

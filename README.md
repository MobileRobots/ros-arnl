rosarnl
========

The rosarnl package contains a ROS node also called rosarnl which provides a
ROS interface to basic ARNL features. Requires ARNL and ArnlBase to be
installed separately, they cannot be automatically installed by rosdep/catkin.
 
In addition, ARNL ArNetworking services are provided, so ARNL can be accessed
and configured via MobileEyes or other ArNetworking clients. Use MobileEyes
to perform scanning for map generation, and use Mapper3 to process the scan
and upload the map to ARNL.

rosarnl is self-contained. It does not depend on any other ROS nodes. It will
connect directly to the robot and laser rangefinder(s), according to ARNL's
robot parameters.

This is intended for simple operation of ARNL navigation, and as a base for
further features to be added.

Some ROS navstack interface compatibiliy is provided (see below).  Some tasks
will need to be performed by MobileEyes and Mapper3 as well. 

See LICENSE.txt for the license terms for this code.  Note that it is not
BSD or GPL, and has additional restrictions on use.
ARNL and ArnlBase are provided under their own MobileRobots Individual 
Software License, and you must have purchased ARNL license(s) to use
the ARNL and ArnlBase libraries.

Topic and service interface
---------------------------
The rosarnl node provides a subset of the standard ROS navigation topic
interface:
 * `/rosarnl_node/move_base_simple/goal`       Publish a PoseStamped message to
    this topic to set a goal. ARNL will begin navigating to this goal if
    possible. 
 * `/rosarnl_node/goalname`   Publish a string message to this topic to set a goal by
   name from the ARIA map.
 * `/rosarnl_node/amcl_pose`  Subscribe to this topic to receive current
   localized position of robot in map as PoseWithCovarianceStamped messages.
 * `/rosarnl_node/initialpose` Publish a PoseWithCovarianceStamped message to
   this topic to change position of robot from which ARNL will continue
   localizing.
 * `/rosarnl_node/global_localization` Service which when called, performs an
   intial localization, assuming robot is either at last known position (as
   stored by ARNL), or at a "home" position in the map. (Note, this differs from
   the `amcl` node, which tries many possible positions across whole map.)
 * `/rosarnl_node/enable_motors` and `/rosarnl_node/disable_motors`: Services
  which  enable/disable the robot motors.
 * `/rosarnl_node/motors_state`: Subscribe to this topic to receive current
   state of motors as a Bool message which is true if enabled, false if disabled.
 * `/rosarnl_node/current_goal`: ARNL's most recently requested goal point, as a Pose.

Planned but not yet implemented:
  * move_base current_goal
  * status message containing mode, status, and detailed status (from ArNetworking server)

The rosarnl node dosen't provide map data or a map reference frame. This may be added
in the future.

`move_base`-compatible actionlib interface
------------------------------------------
Goals can be sent, cancelled, etc to `rosarnl_node/move_base/goal` via actionlib as pose messages,
similar to `move_base` from the standard ROS navigation stack.

`rostopic` and `rosservice` examples
------------------------------------
To quickly test or try out rosarnl, you can use the `rostopic` tool.
`rosarnl_node` must be running, and a map must have been loaded into ARNL using
MobileEyes.

List all available topics in the ROS master:
  rostopic list

Trigger initial localization:
  rosservice call /rosarnl_node/global_localization
Check the `rosarnl_node` log and MobileEyes to see if successful.

Monitor current position:
   rostopic echo /rosarnl_node/amcl_pose

Send to a goal point using move_base_simple interface (no actionlib):
  rostopic pub -1 /rosarnl_node/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}'
Check the log and MobileEyes to see if successful.

Check if motors are enabled:
  rostopic echo /rosarnl_node/motors_state

Enable motors if disabled:
   rosservice call /rosarnl_node/enable_motors

Send to a goal using goalname:
  rostopic pub -1 /rosarnl_node/goalname std_msgs/String "GoalName"


tf
--
Transform (tf) data are provided for the robot base (in the map).


TODO
----

More testing.  

Provide laser and sonar data.
Provide teleoperation (velocity) command interface.
(maybe refactor rosarnl and rosaria to easily extend/incorporate everything from
rosaria?)

Map?

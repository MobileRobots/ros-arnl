rosarnl
========

The rosarnl package contains a ROS node called `rosarnl_node` which provides a
ROS interface to basic ARNL features. 

rosarnl requires ARNL and ArnlBase 1.9.2 or later to be installed.  The packages 
from Adept MobileRobots must be installed  separately, they cannot currently 
be downloaded rosdep/catkin. Download from
<http://robots.mobilerobots.com/wiki/ARNL>.     "Prerelease" testing packages
for 1.9.2 can be found at
<http://robots.mobilerobots.com/ARNL/download/prerelease> and
<http://robots.mobilerobots.com/BaseArnl/download/prerelease>.

To build, check out the repository in the `src` subdirectory of your ROS
(catkin) workspace.  Run `catkin_make` from the workspace directory.

To run `rosarnl_node`, use rosrun: `rosrun rosarnl rosarnl_node`. It will
connect to the robot and begin running.  
 
In addition, ARNL ArNetworking services are provided, so ARNL can be accessed
and configured via MobileEyes or other ArNetworking clients simultaneously 
with the ROS interface. Use MobileEyes to perform scanning for map generation, 
configure ARNL, and more easily do some tasks such as teleoperation and initial
manual localization.  Use Mapper3 to process the scan and upload the map to ARNL.
See the MobileRobots ARNL/Navigation guides, robot manuals, ARNL reference API
documentation, and documentation at <http://robots.mobilerobots.com> for more details
on how to use ARNL.

`rosarnl_node` is self-contained. It does not require any other ROS nodes to be
running. It will connect directly to the robot and laser rangefinder(s), 
according to ARNL's robot parameters.

This is intended for simple operation of ARNL navigation, and as a base for
further features to be added.

Some ROS navstack interface compatibiliy is provided (see below).  Some tasks
will need to be performed by MobileEyes and Mapper3 instead.

See LICENSE.txt for the license terms for this code.  Note that it is not
BSD or GPL, and has additional restrictions on use.
ARNL and ArnlBase are provided under their own MobileRobots Individual 
Software License, and you must have purchased ARNL license(s) with
your robot system in order to use the ARNL and ArnlBase libraries.

Topic and service interface
---------------------------

### Autonomous Navigation (path planning and localization):

The rosarnl node provides a subset of the standard ROS navigation topic
interface (See <http://wiki.ros.org/move_base> and other documentation). 
Note however that by default the topics are prefixed with 
`/ronarnl_node`.  Make sure to use the `rosarnl_node` topic names, or for
transparent compatibility with other ros tools, remap them. For example,
to publish a 2D Nav Goal from rviz, change the topic name in the 2D Nav Goal Tool
Properties in rviz.
 * `/rosarnl_node/move_base_simple/goal`       Publish a PoseStamped message to
    this topic to set a goal. ARNL will begin navigating to this goal if
    possible. 
 * `/rosarnl_node/goalname`   Publish a string message to this topic to set a goal by
   name from the ARIA map.
 * `/rosarnl_node/move_base/goal`, `rosarnl_node/move_base/cancel`,
   `rosarnl_node/move_base/status` and `rosarnl_node/move_base/result`.
   `move_base` compatible actionlib interface to request goals. See
   <http://wiki.ros.org/move_base>.
 * `/rosarnl_node/amcl_pose`  Subscribe to this topic to receive current
   localized position of robot in map as PoseWithCovarianceStamped messages.
 * `/rosarnl_node/initialpose` Publish a PoseWithCovarianceStamped message to
   this topic to change position of robot from which ARNL will continue
   localizing.
 * `/rosarnl_node/global_localization` Service which when called, performs an
   intial localization, assuming robot is either at last known position (as
   stored by ARNL), or at a "home" position in the map. (Note, this differs from
   the `amcl` node, which tries many possible positions across whole map.)
 * `/rosarnl_node/current_goal`: ARNL's most recently requested goal point, as a Pose.
 * `/rosarnl_node/arnl_path_state`: String name indicating changes to the the ARNL path planner internal  state. See `ArPathPlanningInterface::getState` in the ARNL API Reference documentation

The rosarnl node dosen't provide map data. This may be added
in the future.

### Jog Position mode

This mode can be used to do short discrete movements of the robot. 

 * `/rosarnl_node/jog_position_simple/goal`: Publish a `geometry_msgs/Pose2D` message to this topic to move a
    short discrete distance in X or Theta (Y is not yet implemented).  Some
    obstacle sensing is attempted, configure in ARNL configuration (parameter file
    or via MobileEyes).
 * `/rosarnl_node/jog_position/goal`: Initiate an Action Server action with a
    new Jog Position mode goal. Result is sent back when finished, cancel if
    interrupted.

### Laser Data

Laser data is published (e.g. to see it in rviz).  Laser connections are 
configured in the robot parameter file in `/usr/local/Arnl/params`.
See <>. 

Laser data is published similar to rosaria.
For each connected laser, `/rosarnl_node/`_lasername_`_pointcloud` and
`/rosarnl_node/`_lasername_`_laserscan` 
topics are published with `sensor_msgs/PointCloud` and `sensor_msgs/LaserScan`
messages respectively, where _lasername_ is ARIA's identifier for the laser.
For example, `lms2xx_1`, `lms1xx_1`, etc.
 
### Misc

 * `/rosarnl_node/arnl_server_mode` topic: String with the current server mode name
 * `/rosarnl_node/arnl_server_status` topic: String with the current server status message
 * `/rosarnl_node/enable_motors` and `/rosarnl_node/disable_motors` topics: Services
  which  enable/disable the robot motors.
 * `/rosarnl_node/motors_state` topic: Subscribe to this topic to receive current
   state of motors as a Bool message which is true if enabled, false if disabled.
 * `/rosarnl_node/stop` service: Call this service to enter stop mode. Any goals are interrupted and robot is decelerated to a stop.
 * `rosarnl_node/dock` service: Call this service to enter dock mode. The robot stops, then navigates to a docking station to recharge (if the location of one is present in the map as a Dock point. See ARNL configuration e.g. via MobileEyes for more docking parameters.)



Transforms published via `tf`
-----------------------------

ARNL publishes `tf` messages for transform from `map` to `base_link` using the localized robot pose.
This is the same as the pose  (`amcl_pose`) relative to the global map frame.


`move_base`-compatible actionlib interface
------------------------------------------

Goals can be sent, cancelled, etc to `rosarnl_node/move_base/goal` via actionlib as pose messages,
similar to `move_base` from the standard ROS navigation stack.  The `move_base` action
message types are used, install `move_base_msgs` for client API (e.g.
`sudo apt-get install ros-jade-move-base-msgs` for ROS Jade; The `rosarnl`
package also declares a dependency on `move_base_msgs` so you can install with
`rosdep` as well. (`rosdep install rosarnl`). 

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

Send to a goal point using `move_base_simple` interface (no actionlib):

    rostopic pub -1 /rosarnl_node/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}'
Check the ARNL status and pose topics, node log, or MobileEyes to see if successful.

Send to a goal point using `move_base actionlib` interface:

    rostopic pub -1 /rosarnl_node/move_base/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}'
Watch the `/rosarnl_node/move_base/result` topic for status.

Check if motors are enabled:

    rostopic echo /rosarnl_node/motors_state

Enable motors if disabled:

     rosservice call /rosarnl_node/enable_motors

Send to a goal using goalname:

    rostopic pub -1 /rosarnl_node/goalname std_msgs/String "GoalName"

Turn 90 degrees:

    rostopic pub -1 /rosarnl_node/jog_position_simple/goal geometry_msgs/Pose2D '{pose: {orientation: {w: 1.0, z: 1.571}}}'

Move forward half a meter:

    rostopic pub -1 /rosarnl_node/jog_position_simple/goal geometry_msgs/Pose2D '{pose: {position: {x: 0.5}}}'

For examples of sending messages and controlling actions from clients written in
Python, see the client example python scripts in this directory. 

TODO
----

More testing.  

Provide laser and sonar data.
Provide teleoperation (velocity) command interface.
(maybe refactor rosarnl and rosaria to easily extend/incorporate everything from
rosaria?)

Provide map.

Are published tf's useful? should they be configurable?

#! /usr/bin/env python


# This is a simple test or example actionlib client for use with the rosarnl
# move_base-compatible action interface.  For reference, the source code for 
# the Python # SimpleActionClient class will be installed in
# /opt/ros/[rosdistro]/lib/python2.7/dist-packages/actionlib/simple_action_client.py

# TODO test preempting a goal with a new one

import roslib 
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import time

def goal_action_example(x, y, heading = None, canceltime=None):
    # Creates the SimpleActionClient
    move_base_client = actionlib.SimpleActionClient('rosarnl_node/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print 'Waiting for server...'
    move_base_client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    if (heading != None) :
      q = tf.transformations.quaternion_from_euler(0, 0, heading)
      pose.orientation = geometry_msgs.msg.Quaternion(*q)
    goal = MoveBaseGoal()
    goal.target_pose.pose = pose
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()


    # Sends the goal to the action server.
    print 'Sending goal to action server: %s' % goal
    move_base_client.send_goal(goal)

    if canceltime != None:
      print 'Letting action server work for 3 seconds but then cancelling...'
      time.sleep(canceltime)
      print 'Cancelling current goal...'
      move_base_client.cancel_goal()
    else:
      # Waits for the server to finish performing the action.
      print 'Waiting for result...'
      move_base_client.wait_for_result()

    print 'Result received. Action state is %s' % move_base_client.get_state()
    print 'Goal status message is %s' % move_base_client.get_goal_status_text()

    return move_base_client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('goal_action_client_example')
        print 'Testing goal (1.8, 0) with no (invalid) headig...'
        result = goal_action_example(1.8, 0)
        print 'Testing goal (1.8, 2, 3.14159)...'
        result = goal_action_example(1.8, 2, 3.14159)
        print 'Testing goal (1.8, 1, 1.5707)...'
        result = goal_action_example(1.8, 1, 1.5707)
        print 'Testing goal (1.8, 0) but cancelling after 3 secods'
        result = goal_action_example(1.8, 0, heading=None, canceltime=3)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

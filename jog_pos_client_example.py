#! /usr/bin/env python


# This is a simple test or example actionlib client for use with the rosarnl
# jog-position action interface.  For reference, the source code for 
# the Python SimpleActionClient class will be installed in
# /opt/ros/[rosdistro]/lib/python2.7/dist-packages/actionlib/simple_action_client.py
# geomemtry_msgs.msg.Pose2D will be 
# /opt/ros/[rosdistro]/lib/python2.7/dist-packages/geometry_msgs/msg/_Pose2D.py

# TODO test speed, notification of result, cancelling, and preempting a jog with a new one

import roslib 
import rospy
import actionlib
import geometry_msgs
from rosarnl.msg import JogPositionAction, JogPositionGoal
import tf.transformations
import time
import std_msgs

def jog_action_example(x = None, heading = None, canceltime=None):
    # Creates the SimpleActionClient
    jog_position_client = actionlib.SimpleActionClient('rosarnl_node/jog_position', JogPositionAction)

    # Waits until the action server has started up and started
    # listening for jogs.
    print 'Waiting for server...'
    jog_position_client.wait_for_server()

    # Creates a jog movement to send to the action server.
    pose = geometry_msgs.msg.Pose2D()
    if (x != None) :
      pose.x = x
    else:
      pose.x = 0.0
    pose.y = 0.0
    if (heading != None) :
      pose.theta = heading
    jog = JogPositionGoal()
    jog.offset = pose
    jog.timeout = rospy.Duration(10) # 10 second timeout
      
    #jog.target_pose.header.stamp = rospy.Time.now()


    # Sends the jog to the action server.
    print 'Sending jog goal to action server: %s' % jog
    jog_position_client.send_goal(jog)

    if canceltime != None:
      print 'Letting action server work for 3 seconds but then cancelling...'
      time.sleep(canceltime)
      print 'Cancelling current jog goal...'
      jog_position_client.cancel_goal()
      print 'Waiting for result...'
      jog_position_client.wait_for_result()
    else:
      # Waits for the server to finish performing the action.
      print 'Waiting for result...'
      jog_position_client.wait_for_result()

    print '----------------------------------------------------'
    print 'Result received. Action state is %s' % jog_position_client.get_state()
    print 'Goal status message is %s' % jog_position_client.get_goal_status_text()
    print '----------------------------------------------------'

    return jog_position_client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('jog_action_client_example')
        print 'Testing jog forward 0.5 m no heading... '
        result = jog_action_example(x=0.5, heading=None)
        print 'Testing jog 90 deg...'
        result = jog_action_example(x=None, heading=1.571)
        print 'Testing jog -0.5m...'
        result = jog_action_example(x=-0.5, heading=None)
        print 'Testing jog -90 deg...'
        result = jog_action_example(x=None, heading=-1.571)
        print 'Testing combined 0.5m, 90 deg...'
        result = jog_action_example(x=0.5, heading=1.571)
        print 'Testing jog one meter but cancel after 3 seconds...'
        result = jog_action_example(x=1, heading=None, canceltime=3)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

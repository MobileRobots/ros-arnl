#! /usr/bin/env python

import roslib 
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# TODO replace with move_base action definitions?

def goal_action_example():
    # Creates the SimpleActionClient
    move_base_client = actionlib.SimpleActionClient('rosarnl_node/move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print 'Waiting for server...'
    move_base_client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 1.8
    pose.position.y = 0.0
    goal = MoveBaseGoal()
    goal.target_pose.pose = pose
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()


    # Sends the goal to the action server.
    print 'Sending goal to action server: %s' % goal
    move_base_client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print 'Waiting for result...'
    move_base_client.wait_for_result()

    print 'Result received. Action state is %s' % move_base_client.get_state()
    # See
    # http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a460c9f52fd650f918cb287765f169445

    return move_base_client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('goal_action_client_example')
        result = goal_action_example()
        print 'Result'
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

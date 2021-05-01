#! /usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Quaternion
import tf

def move_base_topic(goalx = 0, goaly = 0, goaltheta = 0):
    # listener = tf.TransformListener()
    # start_point_transform = geometry_msgs.msg.TransformStamped()
    # listener.waitForTransform("map","base_link",rospy.Time(0),rospy.Duration(10.0))
    # client=actionlib.SimpleActionClient('move_base',move_base_msgs.msg.MoveBaseAction)
    # client.wait_for_server(rospy.Duration(20))
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = 'map'
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = goalx
    goal.goal.target_pose.pose.position.y = goaly
    goal.goal.target_pose.pose.position.z = 0.0
    q_angle = tf.transformations.quaternion_from_euler(0.0, 0.0, goaltheta)
    q = Quaternion(*q_angle)
    goal.goal.target_pose.pose.orientation = q
    # rospy.loginfo("sending goal")
    # client.send_goal(goal,done_cb=final_cb,active_cb=start_cb,feedback_cb=fb_cb)
    # success = client.wait_for_result(rospy.Duration(30.0))
    # if not success:
    #     client.cancel_goal()
    #     rospy.loginfo("Timed out achieving goal")
    # client.get_result()
    return goal
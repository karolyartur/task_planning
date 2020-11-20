#! /usr/bin/env python

from task_planning_msgs.msg import AddPickAction, AddPickGoal, AddPickResult
import actionlib
import actionlib_tutorials.msg
import rospy


rospy.init_node('add_pick_action_client_test')
client = actionlib.SimpleActionClient('/add_pick', AddPickAction)
client.wait_for_server()
goal = AddPickGoal()

goal.object_name = "object"

client.send_goal_and_wait(goal,rospy.Duration(10),rospy.Duration(10))
client.wait_for_result()

print(client.get_result())

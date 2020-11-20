#! /usr/bin/env python

from task_planning_msgs.msg import ControlTaskAction, ControlTaskGoal, ControlTaskResult
import actionlib
import actionlib_tutorials.msg
import rospy


rospy.init_node('plan_action_client_test')
client = actionlib.SimpleActionClient('/control_task', ControlTaskAction)
client.wait_for_server()
goal = ControlTaskGoal()

goal.data = "10"
goal.operation = 3

client.send_goal_and_wait(goal,rospy.Duration(10),rospy.Duration(10))
client.wait_for_result()

print(client.get_result())

#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Pose
import copy
# !!!function does not avoid obstacles yet !!!

def move_to_bottle(arm, start_pos, bottle_pos, accuracy=.05):
	# start_pos: starting arm position
	# home_pos: predetermined arm position that does not interfere the head camera's view of the AR tags
	# bottle_pos: location of the bottle ar_tag
	# move_to_bottle will path plan to a position (length of gripper) cm away from the bottle
  waypoints = []  
  wpose = Pose()
  wpose.orientation.x = -.5 # not sure about this tbh probably want to test this quaternion
  wpose.orientation.y = -.5
  wpose.orientation.z = -.5
  wpose.orientation.w = .5

  wpose.position.x = start_pos[0]
  wpose.position.y = start_pos[1]
  wpose.position.z = start_pos[2]
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.x = -.4 
  wpose.position.y = .65
  wpose.position.z = .636
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.x = bottle_pos[0] - 0.10 # 10 cm from the bottle, might have to switch signs and change length
  wpose.position.y = bottle_pos[1]
  wpose.position.z = bottle_pos[2]
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = arm.compute_cartesian_path(
                             waypoints,     # waypoints to follow
                             accuracy,     # eef_step
                             0.0,     # jump_threshold
                             avoid_collisions=True) 

  arm.execute(plan) 


def grab_bottle(arm, gripper, start_pos):
	# open gripper
	# moves the arm forward so that the bottle is in between the grippers
	# close gripper 
  gripper.open()
  end_pose = Pose()
  end_pose.orientation.x = 1.0 # not sure about this tbh probably want to test this quaternion
  end_pose.orientation.y = 0.0
  end_pose.orientation.z = 0.0
  end_pose.orientation.w = 0.0

  end_pose.position.x = start_pos[0] + 10 # supposed to move the gripper forward so that the bottle is in between the grippers
  end_pose.position.y = start_pos[1]
  end_pose.position.z = start_pos[2]

  arm.set_pose_target(end_pose)
  arm.set_start_state_to_current_state()
  plan = arm.plan()

  right_plan = arm.plan()
  arm.execute(plan) 

  gripper.close()

#def move_to_cup(home_pos, intermediate_pos, bottle_pos, accuracy):
	# move x cm above the cup (determined by testing) 

#def pour(pour_duration):
	# flips wrist for a specified time that correlates to volume 

#def return_bottle(home_pos, intermediate_pos, bottle_pos, accuracy):
	# home_pos: predetermined arm position that does not interfere the head camera's view of the AR tags
	# intermediate_pos: a postion that may be required to simplify path planning
	# bottle_pos: location of the ar_tag marking the bottle's original location
	# returns bottle to original location

	
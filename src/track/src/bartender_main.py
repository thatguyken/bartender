#!/usr/bin/env python

import rospy
import sys
import pour
import moveit_commander
from intera_interface import gripper as robot_gripper
from geometry_msgs.msg import PoseStamped

def main():
	#Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    #Initialize arm
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)

    '''@KEN call AR tag function here and set values as bottle_pos, find a good home pos, '''
    # home_pos = 
    accuracy = 0.005 # 0.005 is accurate, 0.3 is rough
    pour.move_to_bottle(self.GetEndPointPosition(), home_pos, bottle_pos, accuracy)
    grab_bottle(arm, right_gripper, self.GetEndPointPosition())




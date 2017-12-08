#!/usr/bin/env python
import rospy
import time
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg

class IKmove(object):
    def __init__(self):
        rospy.wait_for_service('compute_ik')
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    def move(self, targetPos, targetOr):

        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = targetPos[0]
        request.ik_request.pose_stamped.pose.position.y = targetPos[1]
        request.ik_request.pose_stamped.pose.position.z = targetPos[2]     
        request.ik_request.pose_stamped.pose.orientation.x = targetOr[0]
        request.ik_request.pose_stamped.pose.orientation.y = targetOr[1]
        request.ik_request.pose_stamped.pose.orientation.z = targetOr[2]
        request.ik_request.pose_stamped.pose.orientation.w = targetOr[3]

        try:
            #Send the request to the service
            response = self.compute_ik(request)
            
            group = MoveGroupCommander("left_arm")
            group.clear_pose_targets()
            group.clear_path_constraints()
            group.set_pose_target(request.ik_request.pose_stamped)
            group.set_start_state_to_current_state()

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            pl = group.plan()
            group.execute(pl)
            group.forget_joint_values("left_w2")   
            group.stop()
            group.clear_pose_targets()
            group.clear_path_constraints()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

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
        #Wait for the IK service to become available
        #rospy.wait_for_service('compute_ik')
        #rospy.init_node('service_query')
        
        #Create the function used to call the service
        #compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        
        #while not rospy.is_shutdown():
        #raw_input('Press [ Enter ]: ')
        
        #2 = .03
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 30
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
            
            #Print the response HERE
            #print(response)
            group = MoveGroupCommander("left_arm")
            #group.set_goal_tolerance(toler)

            group.set_pose_target(request.ik_request.pose_stamped)
            group.set_start_state_to_current_state()

            # orien_const = OrientationConstraint()
            # orien_const.link_name = "left_gripper"
            # orien_const.header.frame_id = "base"
            # # orien_const.orientation.x = 0
            # # orien_const.orientation.y = 0.5
            # # orien_const.orientation.z = 0
            # orien_const.orientation.w = -1.0

            # orien_const.absolute_x_axis_tolerance = 0.1
            # orien_const.absolute_y_axis_tolerance = 0.1
            # orien_const.absolute_z_axis_tolerance = 0.1
            # orien_const.weight = 1.0
            # consts = Constraints()
            # consts.orientation_constraints = [orien_const]
            # group.set_path_constraints(consts)

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without speciftying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])


            # Plan IK and execute
            pl = group.plan()
            group.execute(pl)
            # group.stop()
            # group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

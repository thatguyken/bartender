#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    p1 = [0.354, 0.436, -0.298]
    p2 = [0.507, -0.454, -0.207]

    points = [p1, p2]
    i = 1
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        i = int(not bool(i))
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_wrist"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        ux, uy, uz = points[i]
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = ux
        request.ik_request.pose_stamped.pose.position.y = uy
        request.ik_request.pose_stamped.pose.position.z = uz        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()


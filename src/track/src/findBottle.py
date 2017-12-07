import rospy
import sys
import pour
import time
import tf
#import moveit_commander
#from intera_interface import gripper as robot_gripper
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose,Vector3,Quaternion,PoseStamped
from sensor_msgs.msg import Image


#current_bottle_set = [321, 322, 323]
def callback1(msg):
    # Callback for found_markers
    pass

def GetTargetBottle(self,detectedMarkers):
    orderedMarkers = []
    for num in detectedMarkers:
        if num in current_bottle_set:
            orderedMarkers.append(num)
    orderedMarkers.sort()
    print orderedMarkers
    if not orderedMarkers:
        orderedMarkers.append(-1)
    return orderedMarkers[0]

def FindBottle(self):
    print "==============="
    print "FIND BOTTLE"
    print "==============="
    #Get information about the markers found
    rospy.Subscriber("/track/found_markers",Int16MultiArray, callback1)
    msg1 = rospy.wait_for_message('/track/found_markers',Pose)
    print msg1.data
    detectedMarkers = msg1.data

    chosenMarker = self.GetTargetBottle(detectedMarkers)

    if chosenMarker != -1:
        try:
            msg = Pose()
            position = Vector3()
            orientation = Quaternion()
            targetMarker = 'ar_marker_'+str(chosenMarker)
            print "the marker frame chosen", targetMarker               
            (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))

            Px = trans[0]
            Py = trans[1]
            Pz = trans[2]
            Qx = 1.0
            Qy = 0.0
            Qz = 0.0
            Qw = 0.0

            # Return the position/orientation of the target
            self.Bottle_Position = [Px,Py,Pz] 
            self.Bottle_Quaternion = [Qx,Qy,Qz,Qw]
            print self.Bottle_Position 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    else:
        user_input = raw_input("Drink is ready. Press 's' if finished.")
        if user_input == 's':
            moveit_commander.roscpp_shutdown()
        else:   
            self.FindBottle()

def FindCup(self):
    rospy.Subscriber("/track/found_markers",Int16MultiArray, callback1)
    msg1 = rospy.wait_for_message('/track/found_markers',Pose)
    print msg1.data
    detectedMarkers = msg1.data
    if 340 in detectedMarkers:
        try:
            msg = Pose()
            position = Vector3()
            orientation = Quaternion()
            targetMarker = 'ar_marker_'+str(340)
            print "the cup has been found as", targetMarker               
            (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))

            Px = trans[0]
            Py = trans[1]
            Pz = trans[2]
            Qx = 1.0
            Qy = 0.0
            Qz = 0.0
            Qw = 0.0

            # Return the position/orientation of the target
            self.Cup_Position = [Px,Py,Pz] 
            self.Cup_Quaternion = [Qx,Qy,Qz,Qw]
            print self.Cup_Position 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    else:
        print "the cup is obscured"
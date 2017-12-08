#!/usr/bin/env python
import rospy
import ar_track_alvar_msgs
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout


def callback(msg,prevMarkers):
    #print(msg)
    detectedMarkers = msg.markers
    #print(detectedMarkers)
    now = rospy.get_time() #the current time in seconds
    for detectedMarker in detectedMarkers:
        measuredTime = detectedMarker.header.stamp.secs
        markerID = detectedMarker.id
        prevMarkers[markerID] = measuredTime

    detected_markers = []   
    for marker in prevMarkers.keys():
        if abs(prevMarkers[marker]-int(now)) < 30.0: #if the measurement has been stale for 5 seconds
            detected_markers.append(marker)

    array1 = MultiArrayDimension()
    array1.label = 'numMarkers'
    array1.size = len(detected_markers)
    array1.size = len(detected_markers)
    layout = MultiArrayLayout()
    layout.dim = [array1,]
    layout.data_offset = 0

    msg = Int16MultiArray()
    msg.layout = layout
    msg.data = detected_markers        
    pub.publish(msg)
    
if __name__ == '__main__':
    #This node is responsible for reading the ar_pose_marker
    #topic and reporting back all markers that have been recently
    #detected in the last five seconds" 

    rospy.init_node('publishMarkerInfo')
    foundMarkers = {}
    global pub
    pub = rospy.Publisher('/track/found_markers',Int16MultiArray,queue_size=10)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback, foundMarkers)
    
rospy.spin()
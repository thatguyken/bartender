#!/usr/bin/env python

import rospy
import sys
import pour
import findBottle
from IKmove import IKmove
import time
import tf
import moveit_commander
import baxter_interface
from baxter_interface import gripper as robot_gripper
from baxter_interface import CHECK_VERSION
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose,Vector3,Quaternion,PoseStamped
from sensor_msgs.msg import Image
from copy import deepcopy

import argparse
import baxter_external_devices
import numpy as np

''' CHANGE all orientations self.GetEndPointOr() '''


current_bottle_set = []

#original
#home_trans = [0.747, 0.261, 0.140]
#home_qua = [-0.044, 0.775, -0.044, 0.629]

# home_trans = [0.810, -0.669, 0.436]
# home_qua = [0.821, -0.224, 0.519, 0.085]

home_trans = [0.818, 0.305, 0.192]
home_qua = [0.014, 0.713, -0.050, 0.699]


Bottle_Position = [0.0,0.0,0.0]
Cup_Position = [0.0,0.0,0.0] #9

def callback1(msg):
        # Callback for found_markers
        pass

class makeDrink(object):

    def __init__(self):
        self.current_bottle_set = [10,11,12,13]
        self.Bottle_Position = [0.0,0.0,0.0]
        self.Cup_Position = [0.0,0.0,0.0]

        self.drink_bottle_map = {'2':[10,11], 'manhattan':[10, 11, 12], 'martini':[11, 12, 13], 'white russian':[10, 12, 13], 'rum and coke':[12, 13], 't':[13]}
        #self.cup = 14
        ##### Initialize AR_tag transforms #####
        rospy.init_node('master', anonymous=True)
        self.listener = tf.TransformListener() 
        

        ##### Initialize Inverse Kinematics #####  
        self.Counting = 0
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')
        #rospy.init_node('moveit_node')
        #Initialize MoveIt for arm
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.Arm_left = baxter_interface.Limb('left')
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.left_arm.set_goal_position_tolerance(0.01)
        self.left_arm.set_goal_orientation_tolerance(0.01)
        self.left_arm.set_planner_id('RRTConnectkConfigDefault')
        self.left_arm.set_planning_time(10) 
        ##### Initialize Gripper control #####
        self.left_gripper = robot_gripper.Gripper('left')
        #Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        self.left_gripper.calibrate()

        ##### Verify robot is enabled  #####
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")
        #self.current_bottle_set = []

    def GetEndPointPosition(self):
        rospy.sleep(1.0) # i was tired when I added this
        for i in range(50):
            POSE = self.Arm_left.endpoint_pose()
            EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z]
        return EndPoint

    def GetEndPointOr(self):
        rospy.sleep(1.0) # i was tired when I added this
        for i in range(50):
            POSE = self.Arm_left.endpoint_pose()
            EndPoint = [POSE['orientation'].x ,POSE['orientation'].y, POSE['orientation'].z, POSE['orientation'].w]
        return EndPoint

    def mapDrinktoBottles(self, drink):
        self.current_bottle_set = self.drink_bottle_map[drink]

    def GetTargetBottle(self,detectedMarkers):
        orderedMarkers = []
        for num in detectedMarkers:
            if num in self.current_bottle_set:
                orderedMarkers.append(num)
            else:
                print('current_bottle_set not detected')
        orderedMarkers.sort()
        print orderedMarkers
        if not orderedMarkers:
            orderedMarkers.append(-1)

        #returns the first target marker    
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
                #print(targetMarker.position)              
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

    def returnHome(self):
        print "==============="
        print "HOME IK"
        print "==============="
        ik = IKmove()
        ik.move(home_trans,home_qua)


    def FindCup(self):
        print "==============="
        print "FIND CUP"
        print "==============="
        rospy.Subscriber("/track/found_markers",Int16MultiArray, callback1)
        msg1 = rospy.wait_for_message('/track/found_markers',Pose)
        print msg1.data
        detectedMarkers = msg1.data
        if 9 in detectedMarkers:
            try:
                msg = Pose()
                position = Vector3()
                orientation = Quaternion()
                targetMarker = 'ar_marker_'+str(9)
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

    '''@KEN call AR tag function here and set values as bottle_pos, find a good home pos, '''
    # home_pos = 
    # accuracy = 0.005 # 0.005 is accurate, 0.3 is rough
    # pour.move_to_bottle(self.GetEndPointPosition(), home_pos, bottle_pos, accuracy)
    # grab_bottle(arm, left_gripper, self.GetEndPointPosition())

    def bottleToCup(self):
        print "==============="
        print "BOTTLE TO CUP"
        print "==============="
        ik = IKmove()
        #self.FindBottle()
        #move to cup
        # self.Arm_left.set_joint_position_speed(1.0)
        # cupPos = deepcopy(self.Cup_Position)
        # cupPos[0] -= 0.08 
        # cupPos[2] += 0.20 #above cup
        # ik.move(cupPos,self.GetEndPointOr())
        # currpos = self.GetEndPointPosition()


        # #move up
        # currpos[2] += 0.15
        # ik.move(currpos,self.GetEndPointOr())
        # currpos = self.GetEndPointPosition()


        print "move to the left of the cup"
        cupPos = deepcopy(self.Cup_Position)
        cupPos[1] += 0.17 # might need to increase
        cupPos[2] += 0.20 #above cup
        ik.move(cupPos,self.GetEndPointOr())
        currpos = self.GetEndPointPosition()

        print "move forward, beside cup"
        currpos[0] += 0.09
        ik.move(currpos,self.GetEndPointOr())
        #currpos = self.GetEndPointPosition()




    def pickUpBottle(self):
        print "==============="
        print "PICK UP BOTTLE"
        print "==============="
        ik = IKmove()
        #self.returnHome()
        #ik.move(home_trans,home_qua)
        #print("!!!!CUP!!!!", self.Cup_Position)
        #self.Arm_left.set_joint_position_speed(1.0)
        self.left_gripper.open()
        botpos = deepcopy(self.Bottle_Position)
        #self.FindBottle()
        botpos[0] -= .06
        
        print "move to bottle"
        ik.move(botpos,self.GetEndPointOr())
        currpos = self.GetEndPointPosition()
        rospy.sleep(2.0)

        print "move forward"
        currpos[0] += .105
        ik.move(currpos,self.GetEndPointOr())
        self.left_gripper.close()
        rospy.sleep(2.0)

        print "move up"
        currpos = self.GetEndPointPosition()
        # currpos[0] -= .08
        currpos[2] += .10
        ik.move(currpos,self.GetEndPointOr())

        print "move back"
        currpos = self.GetEndPointPosition()
        #currpos[0] -= .08
        currpos[0] -= .06 #CHECK, might need to increase so that you dont knock over other things (was 0.5)
        ik.move(currpos,self.GetEndPointOr())

        #self.left_gripper.open()
        self.returnHome()


        #ik.move(home_trans,home_qua)
        # self.FindCup()
        # cuppos = self.Cup_Position
        # cuppos[1] += .15
        # ik.move(cuppos,home_qua)

    def pourSequence(self):
        self.returnHome() #delete later
        rospy.sleep(3.0)
        self.FindBottle()
        rospy.sleep(2.0)
        # self.FindCup()
        rospy.sleep(3.0)

        #self.returnHome()
        self.pickUpBottle()
        self.bottleToCup()
        self.move_wrist()
        self.returnHome()
        self.returnBottle()
        self.returnHome()


    def primaryFunction(self):
        #self.returnHome()
        #self.move_wrist()
        #self.returnHomeFK()

        # self.straighten_elbow()

        #self.returnHomeFK() #UPDATE JOANA
        # while not rospy.is_shutdown():
        #     #post = [0.65, 0.123, -0.073]
        #     # USEFUL
        #     # self.pickUpBottle()
        #     # self.bottleToCup()
        #     # self.move_wrist()
        #     # self.returnHome()
        #     # self.returnBottle()
        #     # self.returnHome()

        #     ''' Main sequence of commands '''
        self.FindCup()
        drink = raw_input('Please enter the name of the drink you would like to make\t')
        self.mapDrinktoBottles(drink)

        self.pourSequence()

        #remove that bottle from current bottle set
        self.current_bottle_set = self.current_bottle_set[1:]

        #repeat pour sequence for the rest of the bottles
        while len(self.current_bottle_set) != 0:
            self.pourSequence()
            self.current_bottle_set = self.current_bottle_set[1:]

    def straighten_elbow(self):
        limb = self.Arm_left
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        current_position = [limb.joint_angle(n) for n in names]
        desired_pos = current_position 
        desired_pos[3] -= .52

    # def scanPos(self):



            

    def returnBottle(self):
        print "==============="
        print "RETURN BOTTLE"
        print "==============="
        ik = IKmove()
        #self.Arm_left.set_joint_position_speed(1.0)
        aboveBottle = deepcopy(self.Bottle_Position)


        #move above bottle
        aboveBottle[2] += .1 
        aboveBottle[0] -= .06
        ik.move(aboveBottle,self.GetEndPointOr())
        rospy.sleep(.5)

        #move forward
        currpos = self.GetEndPointPosition()
        currpos[0] += .11
        ik.move(currpos,self.GetEndPointOr())

        #lower bottle into position
        currpos = self.GetEndPointPosition()
        currpos[2] -= .1
        ik.move(currpos,self.GetEndPointOr())

        #let go of bottle
        self.left_gripper.open()
        rospy.sleep(1.0)

        #back away from bottle
        currpos = self.GetEndPointPosition()
        currpos[0] -= .08
        currpos[2] += .10
        ik.move(currpos,self.GetEndPointOr())


    def fk(self, angles):
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        commands = {}
        print(angles)
        for t, n in zip(angles, names):
            commands[n] = t
        print(commands)
        limb = self.Arm_left
        current_position = [limb.joint_angle(n) for n in names]
        diffs = [abs(c - w) for c, w in zip(current_position, angles)]
        while sum(diffs) > 0.1:
            limb.set_joint_positions(commands)
            current_position = [limb.joint_angle(n) for n in names] 
            diffs = [abs(c - w) for c, w in zip(current_position, angles)]
        
    def move_wrist(self):
        print "==============="
        print "MOVE WRIST"
        print "==============="
        limb = self.Arm_left
        self.Arm_left.set_joint_position_speed(0.50) #NEW
        pour_time = 1.0
        deg90 = 1.57
        deg30 = .52
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

        # rotate to 90 deg
        rospy.sleep(2.0)
        current_position = [limb.joint_angle(n) for n in names]
        desired_pos = current_position 
        desired_pos[6] += deg90 + deg30
        self.fk(desired_pos)
        rospy.sleep(2.0)
        self.Arm_left.set_joint_position_speed(1.0)

        #WHY DOES IT TURN BACK ON ITS OWN???????????????????????


        # #rotate 30 deg for set amount of time
        # current_position = [limb.joint_angle(n) for n in names]
        # desired_pos = current_position 
        # desired_pos[6] -= deg30
        # self.fk(desired_pos)
        # rospy.sleep(pour_time)

        # # straighten bottle
        # current_position = [limb.joint_angle(n) for n in names]
        # desired_pos = current_position 
        # desired_pos[6] += (deg90 + deg30)
        # self.fk(desired_pos)
        # rospy.sleep(1.0)

    def returnHomeFK(self):
        limb = self.Arm_left
        names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

        # curr name: ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
        # curr position: [0.0, -0.025310683000109326, -0.011504855909140603, 2.4482333374651204, -0.7251894174728294, -0.48742239535059023, 0.13115535736420286, -1.5715633171886063, -0.06519418348513008, -3.045718854346489, 1.6896798378557834, 0.3160000423043952, 0.6009369736541108, 0.048703890015361885, -1.5558400141127808, -0.006135923151541655, -12.565987119160338]


        # name: ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
        # position: [0.0, -0.0279951493789088, 0.06289321230330196, 2.50767509299568, -0.25579129637989273, -0.7217379607000872, 0.6538593108361576, -1.513655542445932, 0.030679615757708275, -0.01687378866673955, 2.0252381352057176, 0.23623304133435372, -0.863631183579488, -3.0441848735586037, -0.2841699409557729, -2.9272188384823408, -12.565987119160338]




        desired_pos = [-0.7251894174728294, -0.48742239535059023, -0.011504855909140603, 2.4482333374651204, 0.13115535736420286, -1.5715633171886063, -0.06519418348513008]
        self.fk(desired_pos)
        rospy.sleep(3.0)


def main():
    MD = makeDrink()
    MD.primaryFunction()

if __name__ == "__main__":
    main()

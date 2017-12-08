#!/usr/bin/python
rosrun baxter_tools camera_control.py -c right_hand_camera
#rosrun baxter_tools camera_control.py -c head_camera
rosrun baxter_tools camera_control.py -o left_hand_camera

echo "Enable Robot"
rosrun baxter_tools enable_robot.py -e

echo "Running Giant Launch File"
roslaunch track run_robot.launch

#rosrun track bartender_main.py

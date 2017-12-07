#!/usr/bin/python
# echo "Turning on head camera"
#rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
#rosrun baxter_tools camera_control.py -c head_camera
#rosrun baxter_tools camera_control.py -o head_camera
rosrun baxter_tools camera_control.py -o left_hand_camera

echo "Enable Robot"
rosrun baxter_tools enable_robot.py -e

#roslaunch track upload_sawyer.launch

echo "Running Giant Launch File"
roslaunch track run_robot.launch

#rosrun track bartender_main.py

#joint states: rostopic echo /robot/joint_states
#endpoint position: rosrun tf tf_echo base right_gripper
#enable camera: rosrun intera_examples camera_display.py
#display image: rosrun image_view image_view image:=/cameras/head_camera/image

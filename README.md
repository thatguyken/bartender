# bartender
Authors: Kenneth Gourley and Joana Cabrera.
This is a repository of the ROS workspace used to complete our final project.
The project was to make a bartending Baxter robot that could pour mixed drinks.
The important files in this repository are the Python files in src/track/src,
the launch files in src/track/launch, and the bash files in the main directory.
To run this project, you need to run baxter.sh <name> in a terminal window,
where <name> is the name of the robot you wish to connect to. Next, run 
run_bartender.sh, which opens the appropriate cameras, enables the robot,
and calls the launch files. The launch files setup a MoveIt Commander,
the Joint Trajectory Action Server, ar_track_alvar, and an AR Tag publisher
node. The last thing that needs to be done is to run main.py, which takes
care of the logic to pour mixed drinks.

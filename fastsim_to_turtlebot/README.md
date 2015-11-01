fastsim_to_turtlebot
====================

ROS node to use fastsim controllers to control a turtlebot. Just launch fastsim_to_turtlebot.py to have it work.

The bumper and scan are read from turtlebot topics and transmitted to fastsim topics.

The motors are read from fastsim topics and transmitted to turtlebot ones.

turtlebot.launch is to be launched in the turtlebot. It launched all the nodes required to access to the turtlebot, including the kinect and the translation from pointcloud to laserscan.

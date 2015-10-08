#!/usr/bin/env python

# This simple program allows to use a turtlebot instead of fastsim without the need to change anything in the code.

import rospy
from geometry_msgs.msg import Twist

class TranslateFastsimTurtlebot():
    def __init__(self):
        # initiliaze
        self.speed_left=0
        self.speed_right=0

        rospy.init_node('TranslateFastsimTurtlebot', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
        self.sl_fastsim = rospy.Subscriber("/simu_fastsim/speed_left", Float32, self.Speed_left_callback)
        self.sr_fastsim = rospy.Subscriber("/simu_fastsim/speed_right", Float32, self.Speed_right_callback)



        #TurtleBot will stop if we don't keep telling it to move. 
        r = rospy.Rate(20);

        # Twist is a datatype for velocity
        move_cmd = Twist()

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

            # let's go forward at 0.2 m/s
            move_cmd.linear.x = (self.speed_left+self.speed_right)/2.0
            # let's turn at 0 radians/s
            move_cmd.angular.z = (self.speed_right-self.speed_left)/0.5

            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
                        
    def Speed_left_callback(self,data):
        self.speed_left=data

    def Speed_right_callback(self,data):
        self.speed_right=data

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
  # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
  # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
if __name__ == '__main__':
    try:
        TranslateFastsimTurtlebot()
    except:
        rospy.loginfo("Translate Fastsim-Turtlebot node terminated.")

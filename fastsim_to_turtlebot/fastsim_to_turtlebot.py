#!/usr/bin/env python

# This simple program allows to use a turtlebot instead of fastsim without the need to change anything in the code.

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

speed_max=0.2

class TranslateFastsimTurtlebot():
    def __init__(self):
        # initiliaze
        self.speed_left=0
        self.speed_right=0
        self.left_bumper=False
        self.right_bumper=False
        self.lasers=LaserScan()

        rospy.init_node('TranslateFastsimTurtlebot', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        
        # Get speed sent to fastsim and translate it to turtlebot
        self.sl_fastsim = rospy.Subscriber("/simu_fastsim/speed_left", Float32, self.Speed_left_callback)
        self.sr_fastsim = rospy.Subscriber("/simu_fastsim/speed_right", Float32, self.Speed_right_callback)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

     
        # Get bumpers from the turtlebot and translate it to fastsim scripts

        self.sb = rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.Bumper_callback)

        self.laser_tb = rospy.Subscriber("/scan", LaserScan, self.Lasers_callback)

        self.left_bumper_fastsim = rospy.Publisher('/simu_fastsim/left_bumper', Bool, queue_size=10)
        self.right_bumper_fastsim = rospy.Publisher('/simu_fastsim/right_bumper', Bool, queue_size=10)
        self.laser_fastsim = rospy.Publisher('/simu_fastsim/laser_scan', LaserScan, queue_size=10)

 
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)


        #TurtleBot will stop if we don't keep telling it to move. 
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        move_cmd = Twist()



        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            #rospy.loginfo("I am in the loop !")
            
            move_cmd.linear.x = (self.speed_left+self.speed_right)/2.0
            move_cmd.angular.z = (self.speed_right-self.speed_left)/0.5
            print("speed: linear="+str(move_cmd.linear.x)+" angular="+str(move_cmd.angular.z))
            # publish the velocity
            self.cmd_vel.publish(move_cmd)

            # publish the bumpers
            self.left_bumper_fastsim.publish(self.left_bumper)
            self.right_bumper_fastsim.publish(self.right_bumper)
 
            # publish the laser
            self.laser_fastsim.publish(self.lasers)

            # wait 
            r.sleep()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
  # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
  # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
                        

    def Lasers_callback(self,data):
        self.lasers=data
        #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))

    def Speed_left_callback(self,data):
        self.speed_left=float(data.data)
        print("Speed left: "+str(self.speed_left))
	if (abs(self.speed_left)>speed_max):
          rospy.loginfo("Left speed too fast: "+str(self.speed_left))
	  self.speed_left=self.speed_left/abs(self.speed_left)*speed_max
        #rospy.loginfo("Received speed left: "+str(self.speed_left))

    def Speed_right_callback(self,data):
        self.speed_right=float(data.data)
        print("Speed right: "+str(self.speed_right))
	if (abs(self.speed_right)>speed_max):
          rospy.loginfo("Right speed too fast: "+str(self.speed_right))
	  self.speed_right=self.speed_right/abs(self.speed_right)*speed_max

    def Bumper_callback(self,data):
        #rospy.loginfo("Received bumper")
        if (data.bumper==4):
            self.left_bumper=True
        elif (data.bumper==1):
            self.right_bumper=True
        elif (data.bumper==2):
            self.right_bumper=True
            self.left_bumper=True
	else:
            self.right_bumper=False
            self.left_bumper=False



if __name__ == '__main__':
    try:
        TranslateFastsimTurtlebot()
    except:
        rospy.loginfo("Translate Fastsim-Turtlebot node terminated.")

#! /usr/bin/env python

import rospy
import tf
import math

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, TransformStamped, Twist

from tf.transformations import quaternion_about_axis
#from tf.TransformBroadcaster import quaternion_about_axis

class Robot:
    
    def __init__ (self):
        #Node initialisation
        #rospy.init_node('final_assignment_node_adb')
        
        #Setting the frequency of robot to 10MHz
        self.rate = rospy.Rate(10)
        self.start_time = rospy.get_rostime()
        
        #Subscribers
        #self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Int32, self.callback_cmd_vel)
        #Publishers
        self.pub_joint1 = rospy.Publisher("/rms/joint1_position_controller/command", Float64, queue_size=1)
        self.pub_joint2 = rospy.Publisher("/rms/joint2_position_controller/command", Float64, queue_size=1)
        self.pub_joint3 = rospy.Publisher("/rms/joint3_position_controller/command", Float64, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Int32, queue_size=1)

        self.joint1 = Float64()
        self.joint2 = Float64()
        self.joint3 = Float64()

        self.j1_flag = 0
        self.j2_flag = 0
        

    def wave(self):
        #----------------joint1----------------
        if(abs(self.joint1.data) >= 3.0):
            if(self.j1_flag == 0):
                self.j1_flag = 1
            else:
                self.j1_flag = 0

        if(self.j1_flag == 1):
            self.joint1.data += 0.1
        else:
            self.joint1.data -= 0.1
        self.pub_joint1.publish(self.joint1)

        #----------------joint2----------------
        if(self.joint2.data == -1.0):
            self.joint2.data = -1.8
        else:
            self.joint2.data = -1.0
        self.pub_joint2.publish(self.joint2)

        #----------------joint3----------------
        if (self.joint3.data == 0.0):
            self.joint3.data = 0.05
        else:
            self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)

    def zero(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)

    def shutdown_function(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
    
if __name__ == '__main__':
    rospy.init_node('scara0_node')
    scara0 = Robot()
    scara0.zero()
    rospy.sleep(2)
    rospy.on_shutdown(scara0.shutdown_function)
    
    while not rospy.is_shutdown():
        scara0.wave()
        rospy.sleep(1)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

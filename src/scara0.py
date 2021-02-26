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
        self.pub_gripper_right = rospy.Publisher("/rms/gripper_right_position_controller/command", Float64, queue_size=1)
        self.pub_gripper_left = rospy.Publisher("/rms/gripper_left_position_controller/command", Float64, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Int32, queue_size=1)

        self.joint1 = Float64()
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.gripper_right = Float64()
        self.gripper_left = Float64()

        self.j1_flag = 0
        self.j2_flag = 0
        self.last_item = -1

        #Robot spec
        self.L1 = 0.1
        self.L2 = 0.1


    def forward_kinematics(self, theta1, theta2):
        print("inside forward kinematic function")
        self.pub_joint1.publish(theta1)
        self.pub_joint2.publish(theta2)

        x = self.L1*math.sin(theta1) + self.L2*math.sin(theta1 + theta2)
        y = self.L1*math.cos(theta1) + self.L2*math.cos(theta1 + theta2)
        print("x is: " + str(math.floor(1000*x)))
        print("y is: " + str(math.floor(1000*y)))

    def inverse_kinematic(self, x, y):

        #Check for illegal values
        if (abs(x) < 0.2 and abs(y) < 0.2 and y != 0.0):

            #Compute kinematic equation for scara robot
            theta2 = math.acos((math.pow(x, 2) + math.pow(y, 2) - math.pow(self.L1, 2) - math.pow(self.L2, 2)) / (2 * self.L1 * self.L2))

            if (x < 0.0 and y < 0.0):
                theta2 = (-1)*theta2

            theta1 = math.atan2(x, y) - math.atan2((self.L2 * math.sin(theta2)), (self.L1 + self.L2 * math.cos(theta2)))
            #theta1 = math.atan2(y, x) - math.atan2((self.L1 + self.L2 * math.cos(theta2)), (self.L2 * math.sin(theta2)))

            #Correction on when X is negative
            theta2 = (-1)*theta2
            #First quadrant
            if (x > 0.0 and y > 0.0 ):
                theta1 = 1.5708 - theta1
            
            #Second quadrant
            if (x < 0.0 and y >= 0.0 ):
                theta1 = 1.5708 - theta1
            
            #Third quadrant
            if (x < 0.0 and y < 0.0):
                theta1 = -4.7123 + theta1 
            
            #Fourth quadrant
            if (x > 0.0 and y < 0.0):
                theta1 = -1.5708 - theta1
            
            '''
            #First quadrant
            if (x > 0.0 and y > 0.0 ):
                theta1 = 1.5708 - theta1
                theta2 = 1.5708 - theta2

            #Second quadrant
            if (x < 0.0 and y >= 0.0 ):
                theta1 = 1.5708 - theta1

            #Third quadrant
            if (x < 0.0 and y < 0.0):
                theta1 = 4.2123 + theta1

            
            '''
        else:
            print("Value x: " + str(x)  + " and y: " + str(y) + " are illegal")
            theta1 = 0.0
            theta2 = 0.0

        #Displaying and Publishing
        print("For x: " + str(x))
        print("For y: " + str(y))
        print("theta1 is: " + str(theta1))
        print("theta2 is: " + str(theta2))
        self.pub_joint1.publish(theta1)
        self.pub_joint2.publish(theta2)

    def inverse_kinematic2(self, x, y):
        theta2 = math.acos((math.pow(x, 2) + math.pow(y, 2) - math.pow(self.L1, 2) - math.pow(self.L2, 2)) / (2 * self.L1 * self.L2))
        theta1 = math.atan2(y, x) - math.acos((math.pow(x, 2) + math.pow(y, 2) + math.pow(self.L1, 2) - math.pow(self.L2, 2)) / (2 * self.L1 * math.sqrt(math.pow(x, 2) + math.pow(y, 2))))
        print("Coordinates: (" + str(x) + ", " + str(y) +")")
        '''
        print("theta1 is: " + str(theta1))
        print("theta2 is: " + str(theta2))
        '''
        self.pub_joint1.publish(theta1)
        self.pub_joint2.publish(theta2)

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

    def pick(self):
        print("Picking")
        
        self.gripper_right.data = 0.02
        self.gripper_left.data = -0.02
        self.pub_gripper_right.publish(self.gripper_right)
        self.pub_gripper_left.publish(self.gripper_left)
        rospy.sleep(0.5)

        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
        rospy.sleep(0.5)

        self.gripper_right.data = 0.0
        self.gripper_left.data = 0.0
        self.pub_gripper_right.publish(self.gripper_right)
        self.pub_gripper_left.publish(self.gripper_left)
        rospy.sleep(0.5)

        self.joint3.data = 0.05
        self.pub_joint3.publish(self.joint3)
        rospy.sleep(0.5)

    def place(self):
        print("Placing")

        self.gripper_right.data = 0.0
        self.gripper_left.data = 0.0
        self.pub_gripper_right.publish(self.gripper_right)
        self.pub_gripper_left.publish(self.gripper_left)
        rospy.sleep(0.5)

        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
        rospy.sleep(0.5)

        self.gripper_right.data = 0.02
        self.gripper_left.data = -0.02
        self.pub_gripper_right.publish(self.gripper_right)
        self.pub_gripper_left.publish(self.gripper_left)
        rospy.sleep(0.5)

        self.joint3.data = 0.05
        self.pub_joint3.publish(self.joint3)
        rospy.sleep(0.5)
        

    def routine(self):
        basic_pause = 1
        self.last_item += 1
        print("\n-------------- New Item " + str(self.last_item) + " ----------------")
        print("Fetching item to tray")
        self.inverse_kinematic2(0.199, 0.001)
        rospy.sleep(basic_pause)
        self.pick()

        if(self.last_item == 0):
            self.inverse_kinematic2(0.1, 0.1)
            
        if(self.last_item == 1):
            self.inverse_kinematic2(-0.1, 0.1)

        if(self.last_item == 2):
            self.inverse_kinematic2(-0.1, -0.1)

        if(self.last_item == 3):
            self.inverse_kinematic2(0.1, -0.1)

        if(self.last_item == 4):
            self.inverse_kinematic2(0.1, 0.16)

        if(self.last_item == 5):
            self.inverse_kinematic2(0.1, 0.08)

        if(self.last_item == 6):
            self.inverse_kinematic2(0.1, -0.08)

        if(self.last_item == 7):
            self.inverse_kinematic2(0.1, -0.16)
            self.last_item = -1
            
        rospy.sleep(basic_pause)
        self.place()

    def basic_quadrant_ckeck(self):
        print("\nPerforming Quadrant Check")

        basic_pause = 1
        x = 0.1
        y = 0.1
        print("\nFrist Quadrant")
        self.inverse_kinematic2(x, y)
        rospy.sleep(basic_pause)

        print("\nSecond Quadrant")
        self.inverse_kinematic2(-x, y)
        rospy.sleep(basic_pause)

        print("\nThrid Quadrant")
        self.inverse_kinematic2(-x, -y)
        rospy.sleep(basic_pause)

        print("\nFourth Quadrant")
        self.inverse_kinematic2(x, -y)
        rospy.sleep(basic_pause)

    def basic_xy_ckeck(self):
        basic_pause = 5
        print("\nX Positive")
        self.inverse_kinematic(0.199, 0.0001)
        rospy.sleep(basic_pause)

        print("\nX Negative")
        self.inverse_kinematic(-0.199, 0.0001)
        rospy.sleep(basic_pause)

        print("\nY Positive")
        self.inverse_kinematic(0.0, 0.199)
        rospy.sleep(basic_pause)

        print("\nY Negative")
        self.inverse_kinematic(0.0, -0.199)
        rospy.sleep(basic_pause)

    def zero(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
        self.gripper_right.data = 0.0
        self.pub_gripper_left.publish(self.gripper_right)
        self.gripper_left.data = 0.0
        self.pub_gripper_left.publish(self.gripper_left)

    def shutdown_function(self):
        self.joint1.data = 0.0
        self.pub_joint1.publish(self.joint1)
        self.joint2.data = 0.0
        self.pub_joint2.publish(self.joint2)
        self.joint3.data = 0.0
        self.pub_joint3.publish(self.joint3)
        self.gripper_right.data = 0.0
        self.pub_gripper_left.publish(self.gripper_right)
        self.gripper_left.data = 0.0
        self.pub_gripper_left.publish(self.gripper_left)
    
if __name__ == '__main__':
    rospy.init_node('scara0_node')
    scara0 = Robot()
    scara0.zero()
    rospy.sleep(2)
    rospy.on_shutdown(scara0.shutdown_function)
    
    while not rospy.is_shutdown():
        #rospy.sleep(2)
        #scara0.wave()
        #scara0.basic_xy_ckeck()
        #scara0.basic_quadrant_ckeck()
        scara0.routine()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

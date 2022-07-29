#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pr2_tc_description.msg import state
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from math import atan2, fmod, pi
import numpy as np
from pr2_tc_description.srv import stop_pr2

class controller:

    def __init__(self):
        self.goal = None
        self.prev_goal = None
        self.x = None
        self.y = None
        self.theta = None
        self.angle_diff = None
        self.euc_dis = None
        self.msg = None
        self.speed = Twist()
        self.pub = rospy.Publisher("/pr2/cmd_vel", Twist, queue_size = 1)
        self.move_fwd = False
        
    def newOdom(self):
        self.x = self.msg.pose.pose.position.x
        self.y = self.msg.pose.pose.position.y
        rot_q = self.msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
    def ang_euc (self):
        inc_x = self.goal.x - self.x
        inc_y = self.goal.y - self.y
        self.angle_diff = fmod((atan2(inc_y, inc_x) - self.theta), 2*pi)        
        if self.angle_diff < 0.0:
            if abs(self.angle_diff) > 2*pi + self.angle_diff:
                self.angle_diff = 2*pi + self.angle_diff                
        else:
            if self.angle_diff > abs(self.angle_diff - 2*pi):
                self.angle_diff = self.angle_diff - 2*pi        
        p1 = np.array([self.x, self.y])
        p2 = np.array([self.goal.x, self.goal.y])
        self.euc_dis = np.linalg.norm(p1-p2)
        print('Euclidean distance to goal:', self.euc_dis)
        
    def stop_nav (self, bit):
        self.goal = None
        self.prev_goal = None
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.pub.publish(self.speed)
        stopped = bool(1)
        return stopped        
                       
    def control (self, timer):
        self.msg = rospy.wait_for_message('/base_pose_ground_truth', Odometry)
        self.newOdom()        
        if self.goal == None:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.pub.publish(self.speed)
            self.goal = rospy.wait_for_message('/path', state)
            if self.goal == self.prev_goal:
                self.goal = None
            else:
                self.prev_goal = self.goal
        else:
            self.ang_euc()
            if ((self.angle_diff > 0.025 or self.angle_diff < -0.025) and not self.move_fwd) or ((self.angle_diff > 0.05 or self.angle_diff < -0.05) and self.move_fwd):
                print('Adjusting for angle diff:', self.angle_diff)
                self.speed.linear.x = 0.0
                if self.angle_diff > 0:
                    self.speed.angular.z = 150.0*abs(self.angle_diff)
                else: 
                    self.speed.angular.z = -150.0*abs(self.angle_diff)
                self.pub.publish(self.speed)                
            elif self.euc_dis > 1:
                self.move_fwd = True
                print('Moving towards goalpoint:', self.goal) 
                self.speed.linear.x = 150.0*self.euc_dis
                self.speed.angular.z = 0
                self.pub.publish(self.speed)                
            else:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0     
                self.pub.publish(self.speed)
                print('Reached goalpoint:', self.goal,'\n') 
                self.goal = None 
                self.move_fwd = False
        
def controller_loop():
    rospy.init_node("Controller")
    c = controller()
    rospy.Service('stop', stop_pr2, c.stop_nav)
    rospy.Timer(rospy.Duration(.01), c.control)
    rospy.spin()
    
if __name__ == '__main__':
    controller_loop()

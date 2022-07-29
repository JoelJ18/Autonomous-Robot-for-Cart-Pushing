#!/usr/bin/env python3

# -*- coding: utf-8 -*-
# Main script

from PIL import Image
import numpy as np
from RRT import RRT
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from pr2_tc_description.msg import state
from pr2_tc_description.srv import stop_pr2

import matplotlib.pyplot as plt


class navigator:
    def __init__(self):
        self.curpos = None
        self.plan = None
        self.path = []
        self.map_array = []
        self.obstacles = None
        self.travel_node = None
        self.euc = None
        self.complete = False
        self.goal = (10, 60)
        self.msg = None
        self.obs1 = [17.5, 33.5]
        self.euc_obs1 = None
        self.obs1_found = False
        self.obs2 = [43.5, 40.0]
        self.euc_obs2 = None
        self.obs2_found = False        
        self.goalpoint = rospy.Publisher('/path', state, queue_size = 1)
        self.nav_stop = rospy.ServiceProxy('stop', stop_pr2)
             
    def load_map(self, file_path, resolution_scale):
        ''' Load map from an image and return a 2D binary numpy array
            where 0 represents obstacles and 1 represents free space
        '''
        # Load the image with grayscale
        img = Image.open(file_path).convert('L')
        # Rescale the image
        size_x, size_y = img.size
        new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
        img = img.resize((new_x, new_y), Image.ANTIALIAS)

        map_array = np.asarray(img, dtype='uint8')
    
        # Get bianry image
        threshold = 127
        # Result 2D numpy array
        self.map_array = 1 * (map_array > threshold)

    def euc_dist (self):
        p1 = np.array(self.curpos)
        p2 = np.array(list(self.travel_node))
        self.euc = np.linalg.norm(p1-p2)
        print('Euclidean distance to goal:', self.euc) 
    
    def euc_dist_to_obs (self):
        p1 = np.array(self.curpos)
        p2 = np.array(self.obs1)
        self.euc_obs1 = np.linalg.norm(p1-p2)
        p3 = np.array(self.obs2)
        self.euc_obs2 = np.linalg.norm(p1-p3)
        print('Euclidean distance to obs1:', self.euc_obs1)
        print('Euclidean distance to obs2:', self.euc_obs2)
        
    def nav(self, timer):
        self.msg = rospy.wait_for_message('/base_pose_ground_truth', Odometry)
        self.curpos = [self.msg.pose.pose.position.x, self.msg.pose.pose.position.y]
        
        if self.map_array == []:
            print ('Generating the offline map')
            self.load_map("RRT+Replanning+RRN/thick_simplemap.jpg", 0.1)
            
        elif self.path == [] and self.travel_node == None: 
            print ('Finding an offline path from',self.curpos,'to',self.goal) 
            RRT_planner = RRT(self.map_array, self.curpos, self.goal)
            self.plan = RRT_planner.RRT_star(n_pts=1500)
            self.path = self.plan[3]           
            
        elif not self.complete:
            if self.travel_node != None:
                self.euc_dist_to_obs()
                if self.euc_obs1 < 10 and not self.obs1_found and not self.obs2_found:
                    #Stop the robot
                    stop = bool(1)
                    rospy.wait_for_service('stop')
                    self.nav_stop(stop)
                
                    self.load_map("RRT+Replanning+RRN/thick_simplemap2.jpg", 0.1)
                    print ('New obstacle detected, replanning path from',self.curpos,'to',self.goal)
                    RRT_planner = RRT(self.map_array, self.curpos, self.goal)
                    x=None
                    while(x==None):
                        x = RRT_planner.replanner(self.plan[0],self.plan[1],self.plan[2])
                    self.plan=x
                    self.path = self.plan[3]
                    #self.path.pop(0)
                    print('hi')
                    self.obs1_found = True
                    self.travel_node=self.path[0]

                if self.euc_obs2 < 8.5 and not self.obs2_found:
                    #Stop the robot
                    stop = bool(1)
                    rospy.wait_for_service('stop')
                    self.nav_stop(stop)
                    
                    self.load_map("RRT+Replanning+RRN/thick_simplemap3.jpg", 0.1)
                    print ('New obstacle detected, replanning path from',self.curpos,'to',self.goal)
                    RRT_planner = RRT(self.map_array, self.curpos, self.goal)
                    x=None
                    while(x==None):
                        x = RRT_planner.replanner(self.plan[0],self.plan[1],self.plan[2])
                    self.plan=x
                    self.path = self.plan[3]
                    #self.path.pop(0)
                    self.obs2_found = True 
                    print(self.path)
                    print(self.travel_node)
                    self.travel_node=self.path[0]
                
            if self.travel_node == None:
                self.travel_node = self.path[0]
                self.path.pop(0)
                pos = state()
                pos.x = self.travel_node[0]
                pos.y = self.travel_node[1]
                self.goalpoint.publish(pos)
                rospy.sleep(.1)
         
            else:
                self.euc_dist()
                if self.euc < 1.5:
                    if self.travel_node == list(self.goal):
                        self.complete = True
                        print ('Robot has reached the goal.')
                        rospy.signal_shutdown('0')
                    else:
                        self.travel_node = None
                else:
                    pos = state()
                    pos.x = self.travel_node[0]
                    pos.y = self.travel_node[1]
                    self.goalpoint.publish(pos)
        
        
if __name__ == "__main__":
    # Load the map
    #For map resolution 0.8
    #start = (280, 40)
    #goal  = (130, 700)

    #For map resolution 0.5
    #start = (160, 20)
    #goal  = (75, 420)

    #For map resolution 0.3
    #start = (110, 40)
    #goal  = (30, 250)

    #map_array = load_map("RRT+Replanning+RRN/floor_plan.jpg", 0.3)
    #map_array = load_map("RRT+Replanning+RRN/floor_plan.jpg", 0.5)
    #map_array = load_map("RRT+Replanning+RRN/floor_plan.jpg", 0.8)

    # Planning class
    #RRT_planner = RRT(map_array, start, goal)


    # Offline path
    #plan = RRT_planner.RRT_star(n_pts=10000)
    
    
    #Navigator
    rospy.init_node('planner', anonymous=True)
    nav_class = navigator()
    rospy.Timer(rospy.Duration(.1), nav_class.nav)
    rospy.spin()
    

# -*- coding: utf-8 -*-
#!/usr/bin/env python3

# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx
from scipy import spatial

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
        self.validity=1
        

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size
        self.newobsmap = 0
        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.replan = False

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        
        d=math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)  #same as the function in the PRM file,except this takes nodes as inputs
        return d
    
    def check_collision(self, node1, node2):                              #same as the function in the PRM file,except this takes nodes as inputs
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            False if the new node is valid to be connected'''
        

        ax=np.linspace(node1.row,node2.row,dtype=int)
        ay=np.linspace(node1.col,node2.col,dtype=int)
        for i in range(0,len(ax)):
            tempax=ax[i]
            tempay=ay[i]
            if(self.map_array[tempax][tempay]==0 or self.map_array[tempax-2][tempay]==0 or self.map_array[tempax+2][tempay]==0):
                return True

        return False





    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''

        testgoal=np.round(np.random.uniform(1,100))                      #generates a random number from 1-100
        if(testgoal<=goal_bias):                                         #occasionally returns the goal node as the new point, rate is controlled by goal_bias
            return self.goal
        else:
            if(self.replan==False):
                rand_x=int(np.round(np.random.uniform(2,self.size_row-3)))   
                rand_y=int(np.round(np.random.uniform(2,self.size_col-3)))
                return Node(rand_x,rand_y)                               #otherwise returns a new node with random coordinates
            
            else:
                while(True):
                    rand_x=int(np.round(np.random.uniform(2,self.size_row-3)))  #generating a list of points randomly from a unifrom distribution for x and y
                    rand_y=int(np.round(np.random.uniform(3,self.size_col-3)))
                    if(self.newobsmap[rand_x][rand_y]==1):                     #If the coordinates picked are obstacles in the map array then we have our first point
                        break
                    
                while(True):                                      #While loop runs until we find viable coordinates for the second point
                    gaus_x=int(np.round(np.random.normal(rand_x,25)))  #generating points randomly from a gaussian/normal distribution for x and y
                    gaus_y=int(np.round(np.random.normal(rand_y,25)))
                    if(2<gaus_x<self.size_row-3 and 2<gaus_y<self.size_col-3 and self.map_array[gaus_x][gaus_y]!=0): #If the coordinates generated are witin limits and not obstacles in the map array then we add the coordinate to the samples list
                        return Node(gaus_x,gaus_y)     
                

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        dlist=[]
        for i in self.vertices:
            dlist.append(self.dis(i,point))                             #generates a list of the distances of all the nodes to the point and returns the minimum
        return self.vertices[dlist.index(min(dlist))]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        neighbors=[]
        for i in self.vertices:
            if(self.dis(new_node,i)<=neighbor_size and self.check_collision(new_node,i)==False):   #if the distance to the new node from any node in the node list is less than or equal to the neighbor_size
                neighbors.append(i)                                                                #and has no obstacle in the line connecting them, the node in the node list is appended to the neighbors list

        return neighbors


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        clist=[]                                                
                                                                    #collision checks are not needed as they are already made in other functions
        for i in neighbors:
            clist.append(i.cost+self.dis(new_node,i))               #finds the costs to get to the new node from all the neighbors and stores it in clist 

        new_node.parent=neighbors[clist.index(min(clist))]          #sets the parent of the new node to the neighbor through which minimum cost is achieved
        new_node.cost=min(clist)                                    #sets the new cost to the new node

        for i in neighbors:                                         #rewiring of all the neighbors starts
            rcost=new_node.cost+self.dis(new_node,i)                #stores the rewired cost, if a neighbors parent was the new node instead of what it is currently
            if(i.cost>rcost):                                       #if the neighbors current cost is higher than what it is if its connected to the new node, it sets the new node as the parent and gets the rewired cost
                i.parent=new_node
                i.cost=rcost

    def extend(self, snode, rnode, step):                                                     #function to generate a node on the line connecting snode and rnode having a distance of step from snode
        
        d=self.dis(snode,rnode)
        if(d>step):
            d=step

        theta=math.atan2(rnode.col-snode.col,rnode.row-snode.row)
        return Node(int(snode.row+(d*math.cos(theta))),int(snode.col+(d*math.sin(theta))))


    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)
        # Draw Trees or Sample points
        '''for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')'''
        p=[]
        # Draw Final Path if found
        
        if self.found:
            cur = self.goal
            
            while cur.col != self.start.col or cur.row != self.start.row:
                p.append(cur)
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                if(cur in p):
                    print("redo")
                    return None
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')
            p.append(cur)
        '''
        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        #plt.show()
        '''
        return p

    def drawfrompath(self,path,bez_row,bez_col):
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)
        for i in range(0,len(path)-1):
            plt.plot([path[i].col, path[i+1].col], [path[i].row, path[i+1].row], color='b')
            plt.plot(path[i+1].col, path[i+1].row, markersize=3, marker='o', color='b')
        
        for i in range(len(bez_row)):
            for j in range(len(bez_row[0])):
                plt.plot(bez_col[i][j], bez_row[i][j],markersize=1, marker='o', color='r')
            
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')
        plt.show()

                    
        
        
    def RRT_star(self, n_pts=1000, neighbor_size=30,v=[],prevmap=[]):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        self.vertices=v
        if(len(prevmap)!= 0 ):
            self.newobsmap=prevmap+self.map_array
        if(len(self.vertices)==0):
            self.vertices.append(self.start)
        
        #print(self.check_collision(Node(67,104),Node(71,93)))
        goal_bias=10                                                                        #Similar to RRT ,goal bias is set to 8% here it can be changed
        step=10    
        ps=[]                     
        for i in range(0,n_pts):
            newp=self.get_new_point(goal_bias)
            if(self.map_array[newp.row][newp.col]!=0):
                nearn=self.get_nearest_node(newp)
                extn=self.extend(nearn,newp,step)
                if(self.check_collision(nearn,extn)==False):
                    n=self.get_neighbors(extn,neighbor_size)                                #Instead of directly assigning costs and parent to extn node as in RRT, we find the neighbors around extn using neighbor_size as a radius
                    self.rewire(extn,n)                                                     #We rewire extn and the neighbors, it is here the costs and parent nodes are assigned.
                    self.vertices.append(extn)
                    if(extn.row==self.goal.row and extn.col==self.goal.col):                #Again similar to RRT but we do not break when we find the goal, as on further iterations the path will get more optimized through the rewiring
                        self.found=True
                        self.goal=extn                                                 

        # Output
        path = []
        if self.found:   
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
            ps=self.draw_map()
            if(ps==None):
                return None
            new_path = self.rrn(ps)
            
            #Reformatting the path into a better output format
            for n in range(len(new_path)):
                path.append([new_path[n].row, new_path[n].col])
            path.pop(0)
           
            '''
            [bez_row, bez_col]=self.bezier_smoothing(new_path, mtr=50)
            self.drawfrompath(new_path, bez_row, bez_col)
            
            #Reformatting the path into a better output format
            path = [[self.start.row, self.start.col]]
            for n in range(len(bez_row)):
                for m in range(len(bez_row[0])):
                    path.append([bez_row[n][m], bez_col[n][m]])
            path.append([self.goal.row, self.goal.col])
            '''
        
        else:
            print("No path found")
            return None

        
        return [ps,self.vertices,self.map_array,path]

    
    def replanner(self,path,vertices,prevmap):
        valid=[]
        flag=1
        self.replan=True
        for i in range(1,len(vertices)):
            if(vertices[i].parent!=None):
                if(self.check_collision(vertices[i],vertices[i].parent)):
                    vertices[i].validity=0
        
       
        for i in range(0,len(path)):
            if(path[len(path)-1-i].validity==0):
                flag=0
                break

        if(flag==1):
            self.draw_map()
            print('skip replan')
        
        else:
            for i in vertices:
                
                
                xp=i.parent
                if(xp==None):
                    valid.append(i)
                elif(xp.validity==0):
                    i.validity=0
                if(i.validity==1):
                    valid.append(i)

            valid.pop(1)
            
            for i in valid:
                if(i.parent not in valid):
                    i.cost=math.inf
                    i.parent=None
                    
            if(self.start not in valid):
                valid.append(self.start)

            replan = self.RRT_star(400,100,valid,prevmap)

        if(replan==None):
            return None
            
        return [replan[0],replan[1],replan[2],replan[3]]

                    
    def rrn(self,path):
        j=len(path)
        a=path[j-1]        
        newpath=[]
        while(True):

            newpath.append(a)
            xcand=[]
            for i in range(0,j-1):
                if(self.check_collision(path[j-2-i], a)==False):
                    xcand.append(path[j-2-i])
                else:
                    break
                    
            a=xcand[len(xcand)-1]
            j=path.index(a)+1
            if(a==path[0]):
                newpath.append(a)
                break
            
        return newpath

    def bezier_smoothing(self, path_nodes, mtr):
        '''
        This takes in the nodes from the RRT star based path planner and outputs a smooth bezier curve between the nodes
        based on matlab code: https://github.com/MithunNallana/path-smoothing/blob/master/code/matlab/pathsmoothbezier.m
        Translated by: Heather Cummings

        Inputs:
            path_nodes - each of the nodes beginning with start to goal of the found rrt star path
            mtr - the minimum turning radius of the selected robot
        Outputs:
            curves - set of points for curves to smooth the path
            '''
        
        # parameters required for constuction of bezier curve
        c1 = 7.2364
        c2 = (2*(np.sqrt(6)-1))/5
        c3 = (c2+4)/(c1+6)
        
        node_count = len(path_nodes)

        dt    = 0.01
        tBase = np.arange(0.0,1.0,dt)
        tBase3 = (1-tBase)**3
        tBase2 = (1-tBase)**2

        tParm1 = [math.comb(3,0)*(tBase3)]
        tParm2 = [math.comb(3,1)*((tBase)*(tBase2))]
        tParm3 = [math.comb(3,2)*((tBase**2)*((1-tBase)**1))]
        tParm4 = [math.comb(3,3)*(tBase**3)]

        tParmnew1 = []
        tParmnew2 = []
        tParmnew3 = []
        tParmnew4 = []

        ROW = []
        COL = []

        for i in range(len(tParm1[0])):
            if i != 0:
                tParmnew1.append(tParm1[0][i])
                tParmnew2.append(tParm2[0][i])
                tParmnew3.append(tParm3[0][i])
                tParmnew4.append(tParm4[0][i])
        tParm = [tParmnew1, tParmnew2, tParmnew3, tParmnew4]
        tParmRev = np.flipud(tParm)

        for i in range(node_count-2):
            # Waypoint one (W1)
            w1 = np.array([[path_nodes[i].row], [path_nodes[i].col], [0]])
            # Waypoint two (W2)
            w2 = np.array([[path_nodes[i+1].row], [path_nodes[i+1].col], [0]])
            # Waypoint three (W3)
            w3 = np.array([[path_nodes[i+2].row], [path_nodes[i+2].col], [0]])
        
            # ut,un,ub are three perpendicular unit vectors 
            # at w1 where ut(tangent) is pointing towards w2 and
            # un(normal) is in the direction of w3
            ut = (w2-w1)/self.dis(path_nodes[i+1],path_nodes[i])
            up = (w2-w3)/self.dis(path_nodes[i+1],path_nodes[i+2])
            ub = np.cross(up,ut,axis=0)
            ub = ub/np.linalg.norm(ub) #not sure if this will quite work
            un = np.cross(ub,ut,axis=0)
        
            # build transformation matrix to transform coordinates 
            # from global to local and local to global
            rotationMat     = np.concatenate((ut,un,ub), axis=1)
            rotationMatExt  = np.concatenate((rotationMat, [[0, 0, 0]]),axis=0)
            rotationMatExtTranspose  = np.concatenate((np.transpose(rotationMat), [[0, 0, 0]]),axis=0)
            positionMat     = w1
            transformMat    = np.concatenate((rotationMatExt,np.concatenate((w1, [[1]]),axis=0)),axis=1)
            rotMultPos = np.matmul(np.transpose(-rotationMat),positionMat)
            transformMatInv = np.concatenate((rotationMatExtTranspose,np.concatenate((rotMultPos, [[1]]),axis=0)),axis=1)
        
        
            m1 = np.matmul(transformMatInv,np.concatenate((w1, [[1]]),axis=0))
            m2 = np.matmul(transformMatInv,np.concatenate((w2, [[1]]),axis=0))
            m3 = np.matmul(transformMatInv,np.concatenate((w3, [[1]]),axis=0))
        
            m1 = m1[0:-1]
            m2 = m2[0:-1]
            m3 = m3[0:-1]
        
            m1m2 = m2-m1
            m2m3 = m3-m2
        
            m1m2 = m1m2/np.linalg.norm(m1m2)
            m2m3 = m2m3/np.linalg.norm(m2m3)
            u1 = -m1m2
            u2 = m2m3

            gamma = np.arccos(np.sum(np.multiply(m1m2,m2m3)))
            beta = gamma/2
            if(beta == 0):
                beta = 0.00001

            d = ((c2+4)**2/(54*c3))*(np.sin(beta)/(mtr*(np.cos(beta)**2)))
        
            hb = c3*d
            he = hb
            gb = c2*c3*d
            ge = gb
            kb = ((6*c3*np.cos(beta))/(c2+4))*d
            ke = kb
        
            B0 = m2 + d*u1
            B1 = B0 - gb*u1
            B2 = B1 - hb*u1
        
            E0 = m2 + d*u2
            E1 = E0 - ge*u2
            E2 = E1 - he*u2
        
            B2E2 = E2-B2
            ud = B2E2/np.linalg.norm(B2E2)
        
            B3 = B2 + kb*ud
            E3 = E2 - ke*ud

            Bmatrix = np.matmul(transformMat,np.concatenate((np.concatenate((B0,[[1]]),axis=0),np.concatenate((B1,[[1]]),axis=0),np.concatenate((B2,[[1]]),axis=0),np.concatenate((B3,[[1]]),axis=0)),axis=1))
            Ematrix = np.matmul(transformMat,np.concatenate((np.concatenate((E0,[[1]]),axis=0),np.concatenate((E1,[[1]]),axis=0),np.concatenate((E2,[[1]]),axis=0),np.concatenate((E3,[[1]]),axis=0)),axis=1))
        
            Bmatrix   = Bmatrix[0:3,:]
        
            bezierB = np.matmul(Bmatrix,tParm)
            ROW.append(bezierB[0])
            COL.append(bezierB[1])

            Ematrix   = Ematrix[0:3,:]
        
            bezierE = np.matmul(Ematrix,tParmRev)
            ROW.append(bezierE[0])
            COL.append(bezierE[1])

        return ROW, COL
            
                    
            
    

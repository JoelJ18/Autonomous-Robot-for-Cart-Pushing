import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import numpy as np
import time

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs):
        self.row = row             # coordinate
        self.col = col             # coordinate
        self.is_obs = is_obs       # obstacle?
        self.is_dy_obs = is_dy_obs # dynamic obstacle?
        self.tag = "NEW"           # tag ("NEW", "OPEN", "CLOSED")
        self.h = math.inf          # cost to goal (NOT heuristic)
        self.k = math.inf          # best h
        self.parent = None         # parent node


class DStar:
    def __init__(self, grid, dynamic_grid, start, goal):
        # Maps
        self.grid = grid                  # the pre-known grid map
        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)
        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        self.grid_node = [[None for i in range(size_col)] for j in range(size_row)]
        for row in range(size_row):
            for col in range(size_col):
                self.grid_node[row][col] = self.instantiate_node((row, col))

        # The start node
        self.start = self.grid_node[start[0]][start[1]]
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]

        # List
        self.open = set()

        # Result
        self.path = []


    def instantiate_node(self, point):
        ''' Instatiate a node given point (x, y) '''
        row, col = point
        node = Node(row, col, not bool(self.grid[row][col]), not bool(self.dynamic_grid[row][col]))
        return node


    def get_k_min(self):
        '''Get the minimal k value from open list
        
        return:
        k - the minimal k value in the open list; 
            return -1 if the open list is empty
        '''
        # Find the node with minimal k value
        node = self.min_node()
        # If the node is None / open list is empty
        if node == None:
            return -1
        # Get the minimal k value
        else:
            return node.k
    

    def min_node(self):
        '''Get the node with minimal k value from open list
        
        return:
        node - the node with minimal k value in the open list; 
               return None if the open list is empty
        '''
        # If open is empty
        if len(self.open) == 0:
            return None
        # Find the minimum value
        else:
            return min(self.open, key=lambda n: n.k)


    def delete(self, node):
        ''' Remove a node from open list 
            and set it to "CLOSED"
        '''
        self.open.remove(node)
        node.tag = "CLOSED"

    
    def get_neighbors(self, node):
        ''' Get neighbors of a node with 8 connectivity '''
        row = node.row
        col = node.col
        neighbors = []
        # All the 8 neighbors
        for i in range(-1, 2):
            for j in range(-1, 2):
                # Check range
                if row + i < 0 or row + i >= len(self.grid) or \
                   col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                # Do not append the same node
                if i == 0 and j == 0:
                    continue

                neighbors.append(self.grid_node[row + i][col + j])
        
        return neighbors


    def cost(self, node1, node2):
        ''' Euclidean distance from one node to another 

            return:
            distance - Euclidean distance from node 1 to node 2
                       math.inf if one of the node is obstacle
        '''
        # If any of the node is an obstacle
        if node1.is_obs or node2.is_obs:
            return math.inf
        # Euclidean distance
        a = node1.row - node2.row
        b = node1.col - node2.col
        return (a**2 + b**2) ** (1/2)


    def process_state(self):
        ''' Pop the node in the open list 
            Process the node based on its state (RAISE or LOWER)
            If RAISE
                Try to decrease the h value by finding better parent from neighbors
                Propagate the cost to the neighbors
            If LOWER
                Attach the neighbor as the node's child if this gives a better cost
                Or update this neighbor's cost if it already is
        '''

        #### TODO ####
        # Pop node from open list
        min = self.min_node()
        if min is None:
            return -1

        k_old = min.k
        self.delete(min)
        # Get neighbors of the node
        # using self.get_neighbors

        # If node k is smaller than h (RAISE)
        if k_old < min.h:
            for i in self.get_neighbors(min):
                if (i.h < k_old and min.h > (i.h + self.cost(i, min))):
                    min.parent = i
                    min.h = i.h + self.cost(min, i)
        # If node k is the same as h (LOWER)

        if k_old == min.h:
            for i in self.get_neighbors(min):
                if (i.tag == "NEW"  or (i.parent == min and i.h != min.h+self.cost(i, min))) or (i.parent != min and i.h > min.h+self.cost(i, min)) :
                    i.parent = min
                    self.insert(i, min.h+self.cost(min, i))
        
        # Else node k is smaller than h (RASIE)
        else:
            for i in self.get_neighbors(min):
                if i.tag == "NEW" or (i.parent == min and i.h != (min.h+self.cost(i, min))):
                    i.parent = min
                    self.insert(i, min.h+self.cost(i, min))
                else:
                    if (i.parent!=min and i.h > min.h + self.cost(i, min)):
                        self.insert(min, min.h)
                    else:
                        if (i.parent!=min and (min.h > i.h+self.cost(i, min)) and (i.tag == "CLOSED") and (i.h > k_old)):
                            self.insert(i, i.h)         
        #### TODO END ####

        return self.get_k_min()


    def repair_replan(self, node):
        ''' Replan the trajectory until 
            no better path is possible or the open list is empty 
        '''
        #### TODO ####
        # Call self.process_state() until it returns k_min >= h(Y) or open list is empty
        # The cost change will be propagated
        while (len(self.open) != 0):
            k_min = self.process_state()
            if k_min >= node.h:
                break

        #### TODO END ####
        

    def modify_cost(self, obstacle_node, neighbors):
        ''' Modify the cost from the affected node to the obstacle node and 
            put it back to the open list
        ''' 
        #### TODO ####
        # Change the cost from the dynamic obsatcle node to the affected node
        # by setting the obstacle_node.is_obs to True (see self.cost())
        obstacle_node.is_obs = True
        
        # Put the obstacle node and the neighbor node back to Open list 
        for neighbor in neighbors:
            self.insert(neighbor, neighbor.h)
        
        #### TODO END ####

        return self.get_k_min()


    def prepare_repair(self, node):
        ''' Sense the neighbors of the given node
            If any of the neighbor node is a dynamic obstacle
            the cost from the adjacent node to the dynamic obstacle node should be modified
        '''
        #### TODO ####
        # Sense the neighbors to see if they are new obstacles
        for i in self.get_neighbors(node):
            # If neighbor.is_dy_obs == True but neighbor.is_obs == False, 
            # the neighbor is a new dynamic obstacle
            if (i.is_obs == False and i.is_dy_obs == True):
                self.modify_cost(i, self.get_neighbors(i))
                # Modify the cost from this neighbor node to all this neighbor's neighbors
                # using self.modify_cost

        #### TODO END ####


    def insert(self, node, new_h):
        ''' Insert node in the open list

        arguments:
        node - Node to be added
        new_h - The new path cost to the goal

        Update the k value of the node based on its tag
        Append the node t othe open_list
        '''
        # Update k
        if node.tag == "NEW":
            node.k = new_h
        elif node.tag == "OPEN":
            node.k = min(node.k, new_h)
        elif node.tag == "CLOSED":
            node.k = min(node.h, new_h)
        # Update h
        node.h = new_h
        # Update tag and put the node in the open set
        node.tag = "OPEN"
        self.open.add(node)


    def run(self):
        ''' Run D* algorithm
            Perform the first search from goal to start given the pre-known grid
            Check from start to goal to see if any change happens in the grid, 
            modify the cost and replan in the new map
        '''
        start_time = time.time()
        # Search from goal to start with the pre-known map
        self.insert(self.goal,0)
        while (len(self.open) !=0):
            # Process until open set is empty or start is reached
            # using self.process_state()
            self.process_state()  
            if self.start.tag == "CLOSED":
                break
        
        # Visualize the first path if found
        self.get_backpointer_list(self.start)
        if self.path == []:
            print("No path is found")
            return
        else:
            print("The path length is %.2f" %self.start.h)
            print("Runtime is %s seconds" % (time.time() - start_time))
        
        self.draw_path(self.grid, "Path in static map","Static")
        
        # Start from start to goal
        # Update the path if there is any change in the map
        replanner_start_time = time.time()
        cur_node = self.start
        dynamic_path = []
        path_cost = 0
        while (cur_node.parent!=None):
            # Check if any repair needs to be done
            # using self.prepare_repair
            self.prepare_repair(cur_node)

            # Replan a path from the current node
            # using self.repair_replan
            self.repair_replan(cur_node)

            # Get the new path from the current node
            self.get_backpointer_list(cur_node)

            # Uncomment this part when you have finished the previous part
            # for visualizing each move and replanning
            # Visualize the path in progress
            #self.draw_path(self.dynamic_grid, "Path in progress")

            if self.path == []:
                print("No path is found")
                return
            else:
                dynamic_path.append(cur_node)
                path_cost = path_cost + self.cost(cur_node,cur_node.parent)
                

            # Get the next node to continue
            cur_node = cur_node.parent
        self.path = dynamic_path
        print("The path length is %.2f" %path_cost)
        print("Replanner Runtime is %s seconds" % (time.time() - replanner_start_time))
        self.draw_path(self.dynamic_grid, "Path in Dynamic Map", "Dynamic")

        #### TODO END ####
                

    def get_backpointer_list(self, node):
        ''' Keep tracing back to get the path from a given node to goal '''
        # Assume there is a path from start to goal
        cur_node = node
        self.path = [cur_node]
        while cur_node != self.goal and \
              cur_node != None and \
              not cur_node.is_obs:
            # trace back
            cur_node = cur_node.parent
            # add to path
            self.path.append(cur_node)

        # If there is not such a path
        if cur_node != self.goal:
            self.path = []


    def draw_path(self, grid, title="Path", grid_type="Static"):
        '''Visualization of the found path using matplotlib'''
        # Draw map
        row = len(grid)     # map size
        col = len(grid[0])  # map size
        #for i in range(row):
            #for j in range(col):
                #if not self.grid_node[i][j].is_obs: \
                    #ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
                #else:    
                    #ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle      
        
        if grid_type == "Static":
            fig, ax = plt.subplots(1)
            img = 255 * np.dstack((self.grid, self.grid, self.grid))
            ax.imshow(img)

        if grid_type == "Dynamic":
            fig, ax = plt.subplots(1)
            img = 255 * np.dstack((self.dynamic_grid, self.dynamic_grid, self.dynamic_grid))
            ax.imshow(img)

        # Draw path
        for node in self.path:
            row, col = node.row, node.col
            ax.add_patch(Rectangle((col-0.5, row-0.5),1,1,edgecolor='b',facecolor='b'))        # path
        if len(self.path) != 0:
            start, end = self.path[0], self.path[-1]
        else:
            start, end = self.start, self.goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g') #start
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r') #goal
        # Graph settings
        plt.title(title)
        plt.axis('scaled')
        plt.gca().invert_yaxis()
        plt.show()

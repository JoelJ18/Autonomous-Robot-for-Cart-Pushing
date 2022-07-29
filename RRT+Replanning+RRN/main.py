# -*- coding: utf-8 -*-
# Main and helper function

from PIL import Image
import numpy as np
from RRT import RRT


import matplotlib.pyplot as plt


def load_map(file_path, resolution_scale):
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
    map_array = 1 * (map_array > threshold)
    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    #For map resolution 0.8
    #start = (280, 40)
    #goal  = (130, 700)

    #For map resolution 0.5
    start = (160, 20)
    goal  = (75, 420)

    #For map resolution 0.3
    #start = (110, 40)
    #goal  = (30, 250)

    #map_array = load_map("floor_plan.jpg", 0.3)
    map_array = load_map("floor_plan.jpg", 0.5)
    #map_array = load_map("floor_plan.jpg", 0.8)

    # Planning class
    RRT_planner = RRT(map_array, start, goal)


    # Search with RRT and RRT*
    x=RRT_planner.RRT_star(n_pts=10000)
    
    #map_array = load_map("floor_plan_dynamic3.jpg", 0.3)
    map_array = load_map("floor_plan_dynamic3.jpg", 0.5)
    start = (100, 180)
    #map_array = load_map("floor_plan_dynamic3.jpg", 0.8)
    RRT_planner = RRT(map_array, start, goal)
    RRT_planner.replanner(x[0],x[1],x[2])


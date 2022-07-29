# Main and helper function

from PIL import Image
import numpy as np
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

    # Get binary image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    #start = (280, 40)
    #goal  = (130, 700)
    map_array = load_map("floor_plan.jpg", 0.1)
    np.savetxt('simplemap3_array.txt', map_array, delimiter=", ") # map array, 1->free, 0->obstacle

    # Create empty map
    fig, ax = plt.subplots(1)
    img = 255 * np.dstack((map_array, map_array, map_array))
    ax.imshow(img)
    plt.show()

    # Planning class
    #RRT_planner = RRT(map_array, start, goal)

    # Search with RRT*
    #RRT_planner.RRT_star(n_pts=10000)

    #Replanner Function Call

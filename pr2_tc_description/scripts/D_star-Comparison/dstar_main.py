import csv
from search import DStar
from PIL import Image
import numpy as np

# Load map, start and goal point.
def load_map(file_path, resolution_scale):

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
    grid = np.flipud(map_array)
    start = [new_y-50,5]
    goal = [new_y-10,60]

    return grid, start, goal


if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('D_star-Comparison/thick_simplemap.jpg',0.1)
    dynamic_grid, _, _ = load_map('D_star-Comparison/thick_simplemap3.jpg',0.1)
    # grid, start, goal = load_map('map2.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map2.csv')
    # grid, start, goal = load_map('map3.csv')
    # dynamic_grid, _, _ = load_map('dynamic_map3.csv')

    # Search
    d_star = DStar(grid, dynamic_grid, start, goal)

    # Visualize the map
    d_star.draw_path(grid, "static map","Static")
    d_star.draw_path(dynamic_grid, "dynamic map","Dynamic")

    # Run D*
    d_star.run()

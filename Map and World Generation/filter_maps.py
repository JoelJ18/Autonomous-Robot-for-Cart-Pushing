# Main and helper function

from PIL import Image, ImageFilter
import numpy as np
import matplotlib.pyplot as plt


def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    img = img.filter(ImageFilter.MinFilter(5)) # Use 5 for floor plan jpgs and 15 for simplemap jpgs
    img.show()

if __name__ == "__main__":
    map_array = load_map("simplemap.jpg", 0.1)


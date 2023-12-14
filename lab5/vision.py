# This will be the file that will handle the images and convert them to the appropriate structures for the labs
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

class graph:
    def __init__(
            self, width = 0, height = 0, data = [], **kwargs
    ):
        self.width = width
        self.height = height
        self.data = data
        self.kwargs = kwargs

class image:
    def __init__(
            self, data={}, cropx=[], cropy=[], **kwargs
    ):
        self.kwargs = kwargs

        data["Data"] = [int(i) for i in data["Data"]]

        def list_to_matrix(input_list, entries_per_row):
            return [input_list[i:i+entries_per_row] for i in range(0, len(input_list), entries_per_row)]

        self.width = data["Width"]
        self.height = data["Height"]
        
        if not len(cropy) == 2 and len(cropx) == 2:
            assert ValueError("Cropx and cropy must be of shape 2x1")
            
        self.cropx = cropx
        self.cropy = cropy

        # Split data into 3 color streams
        self.r = list_to_matrix(data["Data"][0::3], int(self.width))
        self.g = list_to_matrix(data["Data"][1::3], int(self.width))
        self.b = list_to_matrix(data["Data"][2::3], int(self.width))
    
        # Create 2d matrix for each color
    
    def converter(self):
        r"""
        Converts image to graph structure
        """
    
    def crop(self):
        r"""
        Crops image to specification
        """
        
       # Crop along the x-axis for each color channel
        self.r = self.r[self.cropx[0]:self.cropx[1]]
        self.g = self.g[self.cropx[0]:self.cropx[1]]
        self.b = self.b[self.cropx[0]:self.cropx[1]]

        # Crop along the y-axis for each color channel
        self.r = [row[self.cropy[0]:self.cropy[1]] for row in self.r]
        self.g = [row[self.cropy[0]:self.cropy[1]] for row in self.g]
        self.b = [row[self.cropy[0]:self.cropy[1]] for row in self.b]

    def grayscale(self):
        r"""
        Converts the image to grayscale
        """
        # Iterate through each pixel in the image
        self.gr = np.zeros((len(self.r), len(self.r[0])))

        for i in range(len(self.r)):
            for j in range(len(self.r[0])):
                # Calculate grayscale value by taking the average of RGB channels
                gray_value = (self.r[i][j] + self.g[i][j] + self.b[i][j]) // 3
                
                # Assign the grayscale value to each channel
                self.gr[i][j] = gray_value

    def showcolor(self):
        """
        Displays the color image using Matplotlib
        """
        height = len(self.r)
        width = len(self.r[0])
        
        # Combine the RGB channels into a single image
        rgb_image = [[[self.r[i][j], self.g[i][j], self.b[i][j]] for j in range(width)] for i in range(height)]

        # Display the image using Matplotlib
        plt.imshow(rgb_image)
        plt.axis('off')  # Hide axis ticks and labels
        plt.title('Color Image')
        plt.show()

    def showbnw(self, img):
        """
        Displays black and white image using Matplotlib
        black = 0
        white = 1
        """
        # Display the image using Matplotlib
        plt.imshow(img, cmap='gray')
        plt.axis('off')  # Hide axis ticks and labels
        plt.title('Black and White Image')
        plt.show()
    
    def filter(self, r=[],g=[],b=[]):
        """
        Filter image based on rgb values, returns 2d matrix of filtered values  
          r = [min_val, max_val]
          g = [min_val, max_val]
          b = [min_val, max_val]
        """
        filtered = np.zeros((len(self.r), len(self.r[0])))

        for i in range(len(self.r)):
            for j in range(len(self.r[0])):
                if r[0] <= self.r[i][j] <= r[1] and g[0] <= self.g[i][j] <= g[1] and b[0] <= self.b[i][j] <= b[1]:
                    filtered[i][j] = 1
        
        return filtered

    def create_scale(self, filtered_img):
        r"""
        Finds corners to give scalex, scaley
        """
        potential_corners=[]
        for i, row in enumerate(filtered_img[:-1]):
            for j, index in enumerate(row[:-1]):
                box = filtered_img[i][j]+filtered_img[i][j+1]+filtered_img[i+1][j]+filtered_img[i+1][j]
                if box ==4:
                    # Corner found
                    potential_corners.append((j,i))
        
        if len(potential_corners)<2:
            assert IndexError("potential corners found <0")
        first_corner = potential_corners[0]
        last_corner = (potential_corners[-1][0] + 1, potential_corners[-1][1] + 1)
        return first_corner, last_corner

    def find_square(self, filtered_img=[], size=0, threshold=0.0):
        rows = len(filtered_img)
        cols = len(filtered_img[0])
        
        max_possible_sum = size * size  # For binary matrix, max possible sum for a square of size 'size'
        threshold = max_possible_sum * threshold
        rows = len(filtered_img) - size
        cols = len(filtered_img[0]) - size

        for i in range(rows):
            for j in range(cols):
                box = filtered_img[i:i+size, j:j+size]
                if np.sum(box) >= threshold:
                    return (j + size // 2, i + size // 2)

        return None  # Return None if no square is found above the threshold

        


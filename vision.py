import numpy as np
import cv2
from cv_utils import *
from filterColor import createMask, transformFromBGR


# Detect the plants in the image using color histograms. Return a mask
#  (black/white image, where 0 indicates no plants and 255 indicates plants)
def classifyFoliage(image):
    #foliage_mask = np.zeros(image.shape[0:2], np.uint8)
    # Create a mask that has 255 where there is part of a plant in the image
    #   and 0 everywhere else
    # BEGIN STUDENT CODE
    x_rVals = [(22, 83), (0, 255), (24, 176)]
    x_color_space = 'HSV'
    x_image = transformFromBGR(image, x_color_space)
    x_image = cv2.blur(x_image, (3, 3))
    x_foliage_mask = createMask(x_image, x_rVals, x_color_space)

    #y_rVals = [(0, 87), (33, 255), (15, 200)]
    #y_color_space = 'BGR'
    #y_image = transformFromBGR(image, color_space)
    #y_image = cv2.blur(image, (3, 3))
    #y_foliage_mask = createMask(y_image, y_rVals, y_color_space)
    # END STUDENT CODE
    #foliage_mask = y_foliage_mask & x_foliage_mask
    return x_foliage_mask

# Return the bounding box of the measuring stick
# You can either use the image to find the stick or determine the corners by hand,
#   using some of the utilities we've provided
def findStick (image):
    boundingBox = np.array([[0,0]])
    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return boundingBox

# Find the height of the tallest that occludes (is in front of) the measuring stick.
# Use the bounding box returned by "findStick" to create a mask, 


# Given the foliage mask (as would be returned from classifyFoliage), 
#   return the height in cms the tallest plant that crosses in front 
#   of the measuring stick. Return None if no foliage overlaps the stick
def measureHeight(image, foliage_mask):
    # First, use the bounding box returned from findStick to create a mask
    #   of the measuring stick
    stick_mask = np.zeros(foliage_mask.shape[0:2], np.uint8)
    # BEGIN STUDENT CODE
    # END STUDENT CODE

    # Find the maximum height of plants that overlap the measuring stick
    #   in the foliage_mask
    height = top_row = None
    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return height, top_row

# Use the color calibration squares to find a transformation that will
#   color-correct the image such that the mean values of the calibration
#   squares are the given "goal" colors.
# Return the color-corrected image
def colorCorrect(image, blue_goal, green_goal, red_goal):
    # Find a transform c' = T c, c is the pixel value in the image,
    #   c' is the transformed pixel, and T is the 3x3 transformation matrix
    # Do this by solving d = A x, as per the lecture notes.
    # Note that while the lecture notes describe an affine (3x4) transform,
    #  here we have only 3 colors, so it has to be a Euclidean (3x3) tranform
    # BEGIN STUDENT CODE
    # END STUDENT CODE

    # Fill in the rows of the "A" matrix, according to the notes
    A = np.zeros((9, 9), np.float64)
    # BEGIN STUDENT CODE
    # END STUDENT CODE

    # Fill in the "d" vector with the "goal" colors 
    d = np.zeros((1,9))
    # BEGIN STUDENT CODE
    # END STUDENT CODE

    x = np.matmul(np.matmul(np.linalg.pinv(np.matmul(A.T, A)), A.T), d.T)
    T = x.reshape((3,3))

    # Apply the transform to the pixels of the image and return the
    #  new corrected_image
    corrected_image = image.copy()
    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return corrected_image

# Given an image, return three values:
# 1. An image with all non-foliage parts masked out
# 2. The original image with (a) an outline of the stick (from findStick) and
#    (b) a line at the height of the foliage (from measureHeight), if any
# 3. The height of the foliage
def foliageImages (image):
    foliage_mask = classifyFoliage(image)
    height, row = measureHeight(image, foliage_mask)
    foliageImage = None
    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return foliageImage, image, height

# Given an image, return two values:
# 1. The amount of foliage in the image
# 2. An estimate of the plant health (as a string), based on changes from the
#    previous day (changes in both the amount of foliage and plant height).
def plantHealth (image):
    foliage_mask = classifyFoliage(image)
    height, row = measureHeight(image, foliage_mask)
    greenery = 0
    health_msg = ""
    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return greenery, health_msg


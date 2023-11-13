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
    # END STUDENT CODE
    return x_foliage_mask

# Return the bounding box of the measuring stick
# You can either use the image to find the stick or determine the corners by hand,
#   using some of the utilities we've provided
def findStick (image):
    boundingBox = np.array([[0,0]])
    # BEGIN STUDENT CODE
    boundingBox = np.array([[1960,280], [2046, 280], [1920, 2100], [1846, 2100]])
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
    contour = findStick(image)
    
    cv2.drawContours(stick_mask, [contour], 0, 255, -1)
    
    combined_mask = cv2.bitwise_and(foliage_mask, stick_mask)
    height_list = [360, 585, 820, 1020, 1215, 1390, 1560, 1720, 1870, 2000]

    # END STUDENT CODE

    # Find the maximum height of plants that overlap the measuring stick
    #   in the foliage_mask
    # BEGIN STUDENT CODE
    nonzero_points = np.argwhere(combined_mask > 0)

    if len(nonzero_points) == 0: 
        return None, None 
     
    else: 
        #highest_point = nonzero_points[np.argmax(nonzero_points[:, 1])]
        highest_point = nonzero_points[np.argmin(nonzero_points[:, 0])]
        highest_point_row = highest_point[0]
        height_fin = highest_point_row

        #height, top_row = None
        top_row = highest_point_row
        height = 0 
        
        for i in range(len(height_list)):
            if (i > 0 and height_fin >= height_list[i]): 
                stick_diff = height_list[i-1] - height_list[i]

                foilage_height = height_fin - height_list[i]
                #print("foilage_height", foilage_height)
                height = ((9-i) + (foilage_height/stick_diff))
            elif height_fin >= height_list[0] and height_fin < height_list[1]:
                stick_diff = height_list[0] - height_list[1]
                foilage_height = height_fin - height_list[0]
                height = ((9-i) + (foilage_height/stick_diff))
        if height == 0:
            top_row = contour[3][1]
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
    foliage_mask = cv2.cvtColor(foliage_mask, cv2.COLOR_GRAY2BGR)
    foliageImage = cv2.bitwise_and(image,foliage_mask)
    boundingBox = findStick(image)
    #image = cv2.polylines(image, [boundingBox], isClosed=True, color=(255, 0, 0), thickness=10)
    image = cv2.line(image, boundingBox[0], boundingBox[1], color=(255, 0, 0), thickness=10)
    image = cv2.line(image, boundingBox[1], boundingBox[2], color=(255, 0, 0), thickness=10)
    image = cv2.line(image, boundingBox[2], boundingBox[3], color=(255, 0, 0), thickness=10)
    image = cv2.line(image, boundingBox[3], boundingBox[0], color=(255, 0, 0), thickness=10)
    intersection = []
    edges = [(boundingBox[1], boundingBox[2]), (boundingBox[3], boundingBox[0])]
    for pt1, pt2 in edges:
        if min(pt1[1], pt2[1]) <= row <= max(pt1[1], pt2[1]):
    	    inter = int(pt1[0]+(row-pt1[1])*(pt2[0]-pt1[0])/(pt2[1]-pt1[1]))
    	    intersection.append(inter)
    if len(intersection) == 0:
        start_point = boundingBox[3][0], row
        end_point = boundingBox[1][0], row
    else:
        start_point = (intersection[1]-20, row)
        end_point = (intersection[0]+20, row)
    image = cv2.line(image, start_point, end_point, color=(0, 0, 255), thickness=10)
    # END STUDENT CODE
    return foliageImage, image, height

last_height = 0
last_greenery = 0
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
    global last_height, last_greenery
    greenery = cv2.countNonZero(foliage_mask)/(image.shape[0]*image.shape[1])
    if last_height == 0 or last_greenery == 0:
        if height > 0 or greenery > 0:
            health_msg = "good"
        elif height < 0 or greenery < 0:
            health_msg = "bad"
        else:
            health_msg = "ok"
    else:
        height_change = (height-last_height)/last_height
        volumn_change = (greenery-last_greenery)/last_greenery
        if height_change > 0.05 or volumn_change > 0.05:
    	    health_msg = "good"
        elif height_change < 0 or volumn_change < 0:
    	    health_msg = "bad"
        else:
    	    health_msg = "ok"
    last_greenery = greenery
    last_height = height
    # END STUDENT CODE
    return greenery, health_msg


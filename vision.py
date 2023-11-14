import numpy as np
import cv2
from cv_utils import *
from filterColor import createMask, transformFromBGR

def gammaCorrection(image, gamma):
    table = np.array([int(((i/255.)**gamma)*255) for i in range(256)], np.uint8)
    return cv2.LUT(image, table)

# Detect the plants in the image using color histograms. Return a mask
#  (black/white image, where 0 indicates no plants and 255 indicates plants)
def classifyFoliage(image):
    foliage_mask = np.zeros(image.shape[0:2], np.uint8)
    # Create a mask that has 255 where there is part of a plant in the image
    #   and 0 everywhere else
    # BEGIN STUDENT CODE
    # x_rVals = [(22, 83), (0, 255), (30, 176)]
    # x_color_space = 'HSV'
    # x_image = transformFromBGR(image, x_color_space)
    # x_image = cv2.blur(x_image, (3, 3))
    # kernel = np.ones((50, 50), np.uint8)
    # x_image = cv2.erode(x_image, kernel, iterations=1)
    # x_image = cv2.dilate(x_image, kernel, iterations=1)
    # x_foliage_mask = createMask(x_image, x_rVals, x_color_space)
    image = gammaCorrection(image, 1.6)
    x_rVals = [(22, 68), (140, 255), (10, 235)]
    x_color_space = 'HSV'
    image = cv2.filter2D(image, ddepth=-1, kernel=(np.ones((7,7))/49))
    x_image = transformFromBGR(image, x_color_space)
    foliage_mask = createMask(x_image, x_rVals, x_color_space).astype(np.uint8)
    foliage_mask = cv2.erode(foliage_mask, kernel=np.ones((10,10))) # 3,3 8, 8
    foliage_mask = cv2.dilate(foliage_mask, kernel=np.ones((17,17)))
    # END STUDENT CODE
    return foliage_mask

# Return the bounding box of the measuring stick
# You can either use the image to find the stick or determine the corners by hand,
#   using some of the utilities we've provided
def findStick (image):
    boundingBox = np.array([[0,0]])
    # BEGIN STUDENT CODE
    # boundingBox = np.array([[1960,280], [2053, 280], [1938, 2100], [1848, 2100]])
    boundingBox = np.array([[1465, 585], [1523, 585], [1508, 1765], [1465, 1765]])
    # END STUDENT CODE
    return boundingBox

# Find the height of the tallest that occludes (is in front of) the measuring stick.
# Use the bounding box returned by "findStick" to create a mask, 

# Given the foliage mask (as would be returned from classifyFoliage), 
#   return the height in cms the tallest plant that crosses in front 
#   of the measuring stick. Return None if no foliage overlaps the stick
def measureHeight(image, foliage_mask):
    #   of the measuring stick
    stick_mask = np.zeros(foliage_mask.shape, np.uint8)
    contour = findStick(image)
    cv2.drawContours(stick_mask, [contour], 0, 255, -1)
    
    intersection = cv2.bitwise_and(foliage_mask, stick_mask)
    intersection = cv2.erode(intersection, kernel=np.ones((25,25)), iterations=1)
    # height_list = [360, 585, 820, 1020, 1215, 1390, 1560, 1720, 1870, 2000]
    height_list = [730, 860, 990, 1115, 1235, 1350, 1460, 1565, 1670, 1765]

    if not np.any(intersection):
        return None,None
    nonzero = np.argwhere(intersection > 0)
    highest_point = nonzero[np.argmin(nonzero[:, 0])]
    height_row = highest_point[0]
    height = 0 
        
    for i in range(0,len(height_list)):
        if (i >= 1 and height_row >= height_list[i]): 
            stick_diff = height_list[i-1] - height_list[i]
            fol_dist = height_row - height_list[i]
            height = 9 - i + (fol_dist/stick_diff)
        elif height_row >= height_list[0] and height_row < height_list[1]:
            stick_diff = height_list[0] - height_list[1]
            fol_dist = height_row - height_list[0]
            height = (9 + (fol_dist/stick_diff))
    if height == 0:
        height_row = contour[3][1]
  
    return height, height_row
    

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
    image_height, image_width = image.shape[0],image.shape[1]
    red_mask = np.zeros((image_height, image_width), dtype=np.uint8)

    red_square_coords = np.array([
    [1140,244],  # Top-left corner
    [1283,244],    # Top-right corner
    [1283,395],      # Bottom-right corner
    [1140,395],    # Bottom-left corner
    ], dtype=np.int32)

    red_square_coords = red_square_coords.reshape((-1, 1, 2))
    cv2.drawContours(red_mask, [red_square_coords], 0, 255, thickness=cv2.FILLED)
    red_calib = cv2.mean(image, red_mask)[:-1]
    

    green_mask = np.zeros((image_height, image_width), dtype=np.uint8)

    green_square_coords = np.array([
    [1356,244],  # Top-left corner
    [1499,244],    # Top-right corner
    [1499,403],      # Bottom-right corner
    [1356,403],    # Bottom-left corner
    ], dtype=np.int32)

    green_square_coords = green_square_coords.reshape((-1, 1, 2))
    cv2.drawContours(green_mask, [green_square_coords], 0, 255, thickness=cv2.FILLED)
    green_calib = cv2.mean(image, green_mask)[:-1]
    

    blue_mask = np.zeros((image_height, image_width), dtype=np.uint8)

    blue_square_coords = np.array([
    [1572,252],  # Top-left corner
    [1707,252],    # Top-right corner
    [1707,395],      # Bottom-right corner
    [1572,395],    # Bottom-left corner
    ], dtype=np.int32)

    blue_square_coords = blue_square_coords.reshape((-1, 1, 2))
    cv2.drawContours(blue_mask, [blue_square_coords], 0, 255, thickness=cv2.FILLED)
    blue_calib = cv2.mean(image, blue_mask)[:-1]
    
    

    # BEGIN STUDENT CODE
    #need to build out d, A, x 
    # END STUDENT CODE
    all_goals = []
    rg_B, rg_G, rg_R = red_goal[0], red_goal[1], red_goal[2]
    all_goals.extend((rg_B,rg_G, rg_R))
    gg_B, gg_G, gg_R = green_goal[0], green_goal[1], green_goal[2]
    all_goals.extend((gg_B,gg_G,gg_R))
    bg_B, bg_G, bg_R = blue_goal[0], blue_goal[1], blue_goal[2]
    all_goals.extend((bg_B, bg_G, bg_R))


    

    # Fill in the rows of the "A" matrix, according to the notes
    A = np.zeros((9, 9), np.float64)
    # BEGIN STUDENT CODE in RGB, need to convert from BGR
    


    for i in range(3):
        for j in range(3):
            index = i*3 + j 
            if index % 3 == 0:
                A[i,index] = red_calib[0]
            if index % 3 == 1:
                A[i,index] = red_calib[1]
            if index % 3 == 2:
                A[i,index] = red_calib[2]

    for i in range(3,6):
        for j in range(3):
            index = (i-3)*3 + j
            
            if index % 3 == 0:
                A[i,index] = green_calib[0]
            if index % 3 == 1:
                A[i,index] = green_calib[1]
            if index % 3 == 2:
                A[i,index] = green_calib[2]
    
    for i in range(6,9):
        for j in range(3):
            index = (i-6)*3 + j 
            if index % 3 == 0:
                A[i,index] = blue_calib[0]
            if index % 3 == 1:
                A[i,index] = blue_calib[1]
            if index % 3 == 2:
                A[i,index] = blue_calib[2]

    # END STUDENT CODE
    
    # Fill in the "d" vector with the "goal" colors 
    d = np.zeros((1,9))
    # BEGIN STUDENT CODE
    
    for i in range(len(all_goals)):
        d[0,i] = all_goals[i]
    
    # END STUDENT CODE

    x = np.matmul(np.matmul(np.linalg.pinv(np.matmul(A.T, A)), A.T), d.T)
    T = x.reshape((3,3))
    
    # Apply the transform to the pixels of the image and return the
    #  new corrected_image
    corrected_image = image.copy()

    width = corrected_image.shape[0]
    height = corrected_image.shape[1]
    
    reshaped_array = corrected_image.reshape(-1,3)
    
    reshaped_array_T = np.transpose(reshaped_array)
    result_array = np.matmul(T,reshaped_array_T)
    result_array = np.transpose(result_array)
    result_array = result_array.reshape(width, height, 3)
    
    #norm_image = cv2.normalize(result_array, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)




    # BEGIN STUDENT CODE
    # END STUDENT CODE
    return result_array

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
    if not(height):
        height = 0
    foliageImage = cv2.bitwise_and(image, image, mask=foliage_mask)
    boundingBox = findStick(image)
    image = cv2.line(image, tuple(boundingBox[0]), tuple(boundingBox[1]), color=(255, 0, 0), thickness=5)
    image = cv2.line(image, tuple(boundingBox[1]), tuple(boundingBox[2]), color=(255, 0, 0), thickness=5)
    image = cv2.line(image, tuple(boundingBox[2]), tuple(boundingBox[3]), color=(255, 0, 0), thickness=5)
    image = cv2.line(image, tuple(boundingBox[3]), tuple(boundingBox[0]), color=(255, 0, 0), thickness=5)
    if height == 0:
        row = boundingBox[3][1]
    start_point = (boundingBox[3][0], row)
    end_point = (boundingBox[1][0], row)
    image = cv2.line(image, start_point, end_point, color=(0, 0, 255), thickness=5)
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
    if not(height):
        height = 0
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


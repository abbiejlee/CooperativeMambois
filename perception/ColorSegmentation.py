import cv2
import imutils
import numpy as np
import pdb

def cd_color_segmentation(img):
    """
    Implement the orange color detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the target, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # Find bounding box coordinates
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    LOW_HUE = 3
    HIGH_HUE = 27
    LOW_SATURATION = 159
    HIGH_SATURATION = 255
    LOW_VALUE = 160
    HIGH_VALUE = 255
    ln_color = (0,255,0)
    
    cone_img = img.copy()
     
    #convert to hsv
    hsv = cv2.cvtColor(cone_img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    #apply color ranges
    hue_mask = cv2.inRange(h, LOW_HUE, HIGH_HUE)
    sat_mask = cv2.inRange(s, LOW_SATURATION, HIGH_SATURATION)
    val_mask = cv2.inRange(v, LOW_VALUE, HIGH_VALUE)

    #apply mask
    total_mask = cv2.bitwise_and(hue_mask, cv2.bitwise_and(sat_mask, val_mask))
    cone_img = cv2.bitwise_and(cone_img, cone_img, mask=total_mask)

    #get contours
    _, contours, hierarchy = cv2.findContours(total_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        #get largest contour
        contour_areas = [cv2.contourArea(contour) for contour in contours]
        max_ind = np.argmax(contour_areas)
        biggest_contour = contours[max_ind]
        target_threshold = 0 # the drone has not identify the target

        #if target area detected is large enough, send firing signal 
        biggest_contour_area = contour_areas[max_ind]
        if biggest_contour_area > 3750000: # the drone is too close to the target (within 5'') 
            ln_color = (0, 0, 255)
            target_threshold = 3
            #print("ready to fire")
        elif biggest_contour_area > 842000: # the drone is in the shooting range (5~16'' from target)
            ln_color = (0, 127, 127) 
            target_threshold = 2
        elif biggest_contour_area > 120000: # the drone has identify the target
            ln_color = (127, 127, 0)
            target_threshold = 1
        #draw bounding rectangle
        x1,y1,w,h = cv2.boundingRect(biggest_contour)
        x2,y2 = (x1+w, y1+h)
    cv2.rectangle(cone_img,(x1,y1),(x2,y2),(0,255,0),2)
    ########### YOUR CODE ENDS HERE ###########

    # Return bounding box
    return [((x1, y1), (x2, y2)), ln_color, target_threshold,biggest_contour_area]
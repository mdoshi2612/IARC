#!/usr/bin/env python3
import cv2
import numpy as np
import math

ratio = 1.33
minimum_area = 1000
vertical_fov = 43
horizontal_fov = 70


def color_threshold(bgr):
    img_hsv=cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0+mask1

    # set my output img to zero everywhere except my mask
    output_img = bgr.copy()
    output_img[np.where(mask==0)] = 0

    # or your HSV image, which I *believe* is what you want
    output_hsv = img_hsv.copy()
    output_hsv[np.where(mask==0)] = 0
    return output_img

def final_detection(img, img_mask):
    gray = cv2.cvtColor(img_mask, cv2.COLOR_BGR2GRAY) 
    blur = cv2.medianBlur(gray, 25)
    thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,27,6)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    dilate = cv2.dilate(close, kernel, iterations=2)
    cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
    image = img.copy()
    
    center = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area > minimum_area:
            # Find centroid
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center.append([cX, cY])
            cv2.circle(image, (cX, cY), 20, (36, 255, 12), 2) 
            x,y,w,h = cv2.boundingRect(c)
            cv2.putText(image, 'Radius: {}'.format(w/2), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)
            cv2.drawContours(image, c, -1, (0, 255, 0), 3)
            break

    return thresh, close, image, center


def loc(depth, center):
    center = np.array(center)
    center_depth = center/ratio
    center_depth = np.array(center_depth, dtype=int)
    z = depth[center_depth[1], center_depth[0]]
    angle_x = (float(center[0] - 320)/640)*horizontal_fov
    angle_y = -(float(center[1] - 240)/480)*vertical_fov
    y = int(z*math.tan(math.radians(angle_y)))
    x = int(z*math.tan(math.radians(angle_x)))
    return [x, y, int(z)]
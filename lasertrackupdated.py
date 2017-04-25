from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


#initialization

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
#camera.hflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))

#warmup

time.sleep(0.1)

#capture frames

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #truncate first
    
    rawCapture.truncate(0)
    
    image = frame.array

    blur = cv2.GaussianBlur(image, (5,5),0)

    blur_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    red_min = np.array([0, 100, 80])
    red_max = np.array([10, 256, 256])
    mask1 = cv2.inRange(blur_hsv, red_min, red_max)
     # 170-180 hue
    red2_min = np.array([170, 100, 80])
    red2_max = np.array([180, 256, 256])
    mask2 = cv2.inRange(blur_hsv, red2_min, red2_max)

    # Combine masks to determine object in both ranges 
    mask = mask1 | mask2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode = np.ones((3,3),np.uint8)
    mask = cv2.erode(mask,kern_erode)
    mask = cv2.dilate(mask,kern_erode)
    
    #detect contours, store as best_count

    cnts =  cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        if radius > 0:
            cv2.circle(blur, (int(x), int(y)), int(radius), (0,255,255),2)
            cv2.circle(blur, center, 5, (0,0,255), -1)


## COMMENTED CODE IS FROM PREVIOUS CIRCLE ENCLOSING ALGORITHM

##    image, contours,hierarchy = cv2.findContours(mask, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
##
##    max_area = 0
##    best_cnt = 1
##
##    for cnt in contours:
##        area = cv2.contourArea(cnt)
##        if area > max_area:
##            max_area = area
##            best_cnt = cnt
##            
###find center and draw a circle
##            
##    M = cv2.moments(best_cnt)
##    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
##    
##    cv2.circle(blur,(cx,cy),10,(0,0,255), -1)

    #print centroid coordinates in terminal. These could also be outputted to a file file.
##    print ("cx")   
##    print ("cy")
##    break
##else:
##    cx = 'X coordinate unknown'
##    cy = 'Y coordinate unknown'
##    break 
##    

    

#show the frame
    
    cv2.imshow("Track", blur)
    cv2.imshow('Mask', mask)
    key = cv2.waitKey(1) & 0xFF

#clear stream in preparation for another frame
    
    rawCapture.truncate(0)
  

    if key == ord('e'):
        sys.exit()

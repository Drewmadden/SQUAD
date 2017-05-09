#! /usr/bin/env python
import RPi.GPIO as GPIO
import RPi.GPIO as GPIO2
import RPi.GPIO as GPIO3
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial

## initialize serial connection

arduino = serial.Serial("/dev/ttyACM0", 9600)

time.sleep(2) #wait for init
print("initializing serial connection")



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
    red_min = np.array([0, 100, 80]) #0,100,80
    red_max = np.array([10, 256, 256]) #10, 256, 256
    mask1 = cv2.inRange(blur_hsv, red_min, red_max)
     # 170-180 hue
    red2_min = np.array([150, 100, 80]) #170,100,80
    red2_max = np.array([180, 256, 256]) #180,256,256
    mask2 = cv2.inRange(blur_hsv, red2_min, red2_max)

    # Combine masks to determine object in both ranges 
    mask = mask1 + mask2 #try mask 1 + mask 2
    kern_dilate = np.ones((15,15),np.uint8) #og value 8,8
    kern_erode = np.ones((5,5),np.uint8) #og value 3,3
    mask = cv2.erode(mask,kern_erode)
    mask = cv2.dilate(mask,kern_erode)
    
    #detect contours, store as best_count

    cnts =  cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    cx = None
    cy = None

    #-----------------sensors------------------#
    GPIO.setmode(GPIO.BCM)

    TRIG=18 ##middle sensor
    ECHO=17

    print "distance measurement in progress"

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)


    GPIO.output(TRIG,False)
    
    time.sleep(.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    print"1"

    while GPIO.input(ECHO)==0:
        start = time.time()
    print"a"
    while GPIO.input(ECHO)==1:
        end = time.time()
    print"b"

    pulse_duration = end - start
    distance = pulse_duration * 17150
    distance = distance / 2.54
    distance = round (distance,2)
    print "distance:", distance , "inches"


    GPIO3.setmode(GPIO3.BCM)
    GPIO2.setmode(GPIO2.BCM)

    TRIG=16
    ECHO=19
    TRIG2=23
    ECHO2=22

       

    GPIO3.setup(TRIG,GPIO3.OUT)
    GPIO3.setup(ECHO,GPIO3.IN)



    GPIO3.output(TRIG,False)

    time.sleep(.1)

    GPIO3.output(TRIG, True)


    time.sleep(0.00001)
    GPIO3.output(TRIG, False)

    print"2"

    while GPIO3.input(ECHO)==0:
        pulse_start = time.time()
    print"s"
    while GPIO3.input(ECHO)==1:
        pulse_end = time.time()
    print"r"
    pulse_duration = pulse_end - pulse_start
    distance3 = pulse_duration * 17150
    distance3 = distance3 / 2.54
    distance3 = round (distance3,2)
    print "distance:", distance3 , "inches"

    GPIO2.setup(TRIG2,GPIO2.OUT)
    GPIO2.setup(ECHO2,GPIO2.IN)
    
    GPIO2.output(TRIG2,False)
    
    time.sleep(.1)

    GPIO2.output(TRIG2, True)

    time.sleep(0.00001)

    GPIO2.output(TRIG2, False)


    while GPIO2.input(ECHO2)==0:
        pulse_start = time.time()
    print "x"
    while GPIO2.input(ECHO2)==1:
        pulse_end = time.time()
    print "y"


    pulse_duration = pulse_end - pulse_start
    distance2 = pulse_duration * 17150
    distance2 = distance2 / 2.54
    distance2 = round (distance2,2)
    print "distance2:", distance2 , "inches"

    #-------------end sensors-----------------#

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        while radius > 0:
            cv2.circle(blur, (int(x), int(y)), int(radius), (0,255,255),2)
            cv2.circle(blur, center, 5, (0,0,255), -1)
            print(center)

            if (distance3 > distance2) and (distance < 25) :
                print "Object detected. Turn left."
                arduino.write('a')
                break
                
            elif (distance3 < distance2) and (distance < 25) :
                print "Object detected. Turn right."
                arduino.write('b')
                break
                
            elif (distance3 and distance2 and distance < 10) :
                arduino.write('c')
                break
            else:
                ##turn rights
                if ((cx > 30) and (cx < 70)):
                    print "Turn right"
                    arduino.write("j")
                    break
                elif ((cx > 70) and (cx < 130)):
                    print "Turn right"
                    arduino.write("k")
                    break
                elif ((cx > 0) and (cx < 30)):
                    print "Turn right"
                    arduino.write("a")
                    break

                ##turn lefts
                
                elif ((cx > 600) and (cx < 630)):
                      print "Turn left"
                      arduino.write("b")
                      break
                elif ((cx > 560) and (cx < 600)):
                      print "Turn left"
                      arduino.write("r")
                      break
                elif ((cx > 500) and (cx < 560)):
                      print "Turn left"
                      arduino.write("q")
                      break
                    
                ## let her lay
                elif ((cx > 130) and (cx < 500)):
                      print "Drive forward"
                      arduino.write("d")
                      break
                else:
                      print "stopping"
                      arduino.write("c")
                      break
    else:
        print "no detection"
        arduino.write("c")
                    

                 
                     

            
            


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

#clean up GPIO
    GPIO.cleanup()    
    GPIO2.cleanup()
    GPIO3.cleanup()


    
  

    if key == ord('e'):
        sys.exit()

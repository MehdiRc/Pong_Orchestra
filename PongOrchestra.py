#!/usr/bin/env python
# coding: utf-8

#### #### #### #### ####
#### PONG ORCHESTRA ####
#### #### #### #### ####

#############################################
# BY: MEHDI CHAKHCHOUKH && QUENTIN LEMASSON #  
#############################################


from __future__ import division
import math

try:
    from itertools import izip
except ImportError: # Python 3
    izip = zip
    xrange = range

import cv2
import numpy as np
import socket



## SETTINGS TO SEND DATA TO UNITY GAME#####################
UDP_IP = "127.0.0.1"                                     
UDP_PORT = 5065
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
###########################################################
###########################################################


####################
### COLOR RANGES ###
####################

#Red#====================================================
lower_redH=[0,10]
upper_redH=[170,180]
redS=[100,255]
redV=[100,255]

#Green#==================================================
greenH=[30,80]
greenS=[100,255]
greenV=[100,255]

#Blue#===================================================
blueH =[95,132]
blueS =[100,255]
blueV =[100,255]
##########################################################
##########################################################



## fUNCTION TO SEND DATA AS UDP PAQUETS ON LOCAL ADRESS #############
def send_data(x,y):
    #send data to pureData
    s = socket.socket()
    host = socket.gethostname()
    port = 3000
    try:
        s.connect((host, port))
        mx = str(x)
        my = str(y)
        message = mx + " " + my + " ;"
        s.send(message.encode('utf-8'))
    except:
        print("CONNECTION PROBLEM")
######################################################################
######################################################################

## COLOR CALIBRATION FUNCTION USING MOUSE AND KEYBOARD ############################
## CALLIBRATION IS ADDITIVE ( you need to relaunch the program if you went too far)
###################################################################################

def mouseClick(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        key = cv2.waitKey(0)
        if key == ord('r') or key == ord('g') or key == ord('b'):  
            rgbColor = img[y,x]
            print("BRG: ",rgbColor)
            im = np.uint8([[rgbColor]]) 
            hsvColor = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            print ("HSV",hsvColor[0][0])
            print("Coordinates of pixel: X: ",x,"Y: ",y)
            print("input as,", chr(key) )
            
            if key == ord('r'):
                if hsvColor[0][0][0] <= 100:
                    if hsvColor[0][0][0] not in lower_redH:
                        lower_redH.append(hsvColor[0][0][0])
                else:
                    if hsvColor[0][0][0] not in upper_redH:
                        upper_redH.append(hsvColor[0][0][0])
                    
                        
                if hsvColor[0][0][1] not in redS:
                    redS.append(hsvColor[0][0][1])
                if hsvColor[0][0][2] not in redV:
                    redV.append(hsvColor[0][0][2])
    
                        
            if key == ord('g'):
                if hsvColor[0][0][0] not in greenH:
                    greenH.append(hsvColor[0][0][0])
                if hsvColor[0][0][1] not in greenS:
                    greenS.append(hsvColor[0][0][1])
                if hsvColor[0][0][2] not in greenV:
                    greenV.append(hsvColor[0][0][2])                
            if key == ord('b'):
                if hsvColor[0][0][0] not in blueH:
                    blueH.append(hsvColor[0][0][0])
                if hsvColor[0][0][1] not in blueS:
                    blueS.append(hsvColor[0][0][1])
                if hsvColor[0][0][2] not in blueV:
                    blueV.append(hsvColor[0][0][2])
###################################################################################
###################################################################################


## CAMERA CAPTURE (CAHNGE VALUE TO CHANGE CAMERA IF MULTIPLE)##
cap = cv2.VideoCapture(0)
###############################################################

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")
    
cv2.namedWindow('res')
cv2.setMouseCallback('res',mouseClick)

while True:
    ret, frame = cap.read()
    
    # SET SIZE OF CAMERA FEED
    frame = cv2.resize(frame, None, fx=0.75, fy=0.75, interpolation=cv2.INTER_AREA)

    # GAUSSIAN BLUR: ACTIVATE IF LOTS OF NOISE IN CAMERA FEED ###
    #rrframe = cv2.GaussianBlur(frame, (13, 13), 10)           ##
    #############################################################


    #CONVERT RGB CAMERA FEED TO HSV ( HUE IS BETTER TO DETERMIN NEIGHBOURING COLORS)
    img = frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    
    #####################################################
    ##CREATING THE COLOR MASKS TO SEPARATES THE COLORS ##
    #####################################################

    #Red#====================================================
    # lower mask (0-10)
    l_red = np.array([min(lower_redH),min(redS),min(redV)])
    u_red = np.array([max(lower_redH),max(redS),max(redV)])
    mask0 = cv2.inRange(hsv, l_red, u_red)
    
    # upper mask (170-180)
    l_red = np.array([min(upper_redH),min(redS),min(redV)])
    u_red = np.array([max(upper_redH),max(redS),max(redV)])
    mask1 = cv2.inRange(hsv, l_red, u_red)

    # join masks to create one and only red mask
    redMask = mask0+mask1
    #====================================================
    
    # Green #====================================================    
    lower_range = np.array([min(greenH),min(greenS),min(greenV)])
    upper_range = np.array([max(greenH),max(greenS),max(greenV)])
    greenMask = cv2.inRange(hsv, lower_range, upper_range)
    #====================================================
    
    # Blue ##====================================================
    lower_range = np.array([min(blueH),min(blueS),min(blueV)])
    upper_range = np.array([max(blueH),max(blueS),max(blueV)])
    blueMask = cv2.inRange(hsv, lower_range, upper_range)
    #====================================================
    


    ####################
    ## SHOW THE MASKS ##
    ####################
    cv2.imshow('blueMask', cv2.flip( blueMask,1))
    cv2.imshow('greenMask', cv2.flip(greenMask,1))
    cv2.imshow('redMask', cv2.flip(redMask,1))
    
    # BLOB DETTECTION (TO DETECT CERCLES OF COLORS (THE BALLS)) +++++++++++++++++++++++++++++++++++++++++++++++++
    # Read image
    blobRed = cv2.bitwise_not(redMask)
    blobGreen = cv2.bitwise_not(greenMask)
    blobBlue = cv2.bitwise_not(blueMask)

    
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Filter by Area
    params.filterByArea = True
    params.minArea = 50
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.7
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    redKeypoints = detector.detect(blobRed)
    greenKeypoints = detector.detect(blobGreen)
    blueKeypoints = detector.detect(blobBlue)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    img = cv2.drawKeypoints(img, redKeypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img = cv2.drawKeypoints(img, greenKeypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img = cv2.drawKeypoints(img, blueKeypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
    # Show keypoints (detected blobs)
    cv2.imshow("res", cv2.flip( img, 1))
    #EndBLOB DETECTION ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    
    ## CREATING AND SENDING THE MESSAGE 
    ####################################

    msg = ""
    ## RED PART OF MESSAGE
    if len(redKeypoints) != 0:
        red_f = redKeypoints[0].pt[0] + 200
        red_v = redKeypoints[0].pt[1]
        red_s = redKeypoints[0].size

        msg += "red;"+str(red_f)+";"+str(red_v)+";"+str(red_s)+"*"
    else:
        red_f = -1 
        red_v = -1
        red_s = -1
        msg += "red;"+str(red_f)+";"+str(red_v)+";"+str(red_s)+"*"

    # GREEN PART OF MESSAGE
    if len(greenKeypoints) != 0:

        green_f = greenKeypoints[0].pt[0] + 200
        green_v = greenKeypoints[0].pt[1]
        green_s = greenKeypoints[0].size 

        msg += "green;"+str(green_f)+";"+str(green_v)+";"+str(green_s)+"*"
    else:
        green_f = -1 
        green_v = -1
        green_s = -1

        msg += "green;"+str(green_f)+";"+str(green_v)+";"+str(green_s)+"*"

    # BLUE PART OF MESSAGE
    if len(blueKeypoints) != 0:
        blue_f = blueKeypoints[0].pt[0] + 200
        blue_v = blueKeypoints[0].pt[1]
        blue_s = blueKeypoints[0].size
        msg += "blue;"+str(blue_f)+";"+str(blue_v)+";"+str(blue_s)+"*"
    else:
        blue_f = -1
        blue_v = -1
        blue_s = -1
        msg += "blue;"+str(blue_f)+";"+str(blue_v)+";"+str(blue_s)+"*"
    
    #DISPLAY MESSAGE ON CONSOLE (UNCOMMENT FOR DEBBUGGING)
    #print(msg)
    
    ##SENDING THE MESSAGE AS UDP PAQUETS
    sock.sendto( (str(msg)).encode(), (UDP_IP, UDP_PORT) )

    ## EXIT WINDOW IF ESC IS PRESSED 
    c = cv2.waitKey(1)
    if c == 27:
        break

##CLOSING ALL WINDOWS
cv2.destroyAllWindows()
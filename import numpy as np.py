import cv2
import numpy as np

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm





lowerBound = np.array([98, 50, 50])  # setting the blue lower limit 
upperBound = np.array([139, 255, 255])  # setting the blue upper limit 

cam = cv2.VideoCapture(0)
kernelOpen = np.ones((5, 5))
kernelClose = np.ones((20, 20))

font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, img = cam.read()
    if not ret:
        break

    # Convert BGR to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Create the Mask
    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    # Morphology
    maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
    maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

    maskFinal = maskClose
    conts, _ = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cv2.drawContours(img, conts, -1, (255, 0, 0), 3)
    
    for i in range(len(conts)):
        x, y, w, h = cv2.boundingRect(conts[i])
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)
    
        cv2.putText(img, str(i + 1), (-x, y + h + 3), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        
        X = x - 54
        Y = y - 458
        

		
        coord_text = f"({X},{Y})"
        cv2.putText(img, coord_text, (x + 5, y - 10), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    

    cv2.imshow("maskClose", maskClose)
    cv2.imshow("maskOpen", maskOpen)
    cv2.imshow("mask", mask)
    cv2.imshow("cam", img)
    
    if cv2.waitKey(10) & 0xFF == 27:  # Press 'ESC' to exit
        break




cam.release()
cv2.destroyAllWindows()



            



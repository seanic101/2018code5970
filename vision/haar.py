import numpy as np
import cv2

cap = cv.VideoCapture(0)

while(1):
	_, frame = cap.read() 
#what does this do = wdtd also does _ have to be replaced w variable?
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
#changes color to hsv
	lower_red = np.array([30,150,50]) 
#later change to green for LED ring light
	upper_red = np.array([255,255,180])


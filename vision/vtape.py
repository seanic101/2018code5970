import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(1):
	_, frame = cap.read() 
#sets frame to equal the video capture window
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
#changes color to hsv
	lower_green = np.array([30,130,23]) 
#saw online somewhere that last value only goes to 180 so if problems occur with lower green...
	upper_green = np.array([0,255,0])

	mask = cv2.inRange(hsv, lower_green, upper_green)
#mask filters colors out of the green range from the frame being read
	result = cv2.bitwise_and(frame, frame, mask = mask)

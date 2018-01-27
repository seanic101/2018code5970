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
#applies mask by saying input array 1 and 2 are frame, output array is mask (still don't fully understand why a bitwise_and is necessary)
	
	cv2.imshow('frame', frame)
	cv2.imshow('mask', mask)
	cv2.imshow('result', result)
#why can't you just show result?

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
#forgot how 0xFF works...I think it means something to do with wait for 15 characters or something... then quit
cv2.destroyAllWindows()
cap.release()


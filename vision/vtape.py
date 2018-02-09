import numpy as np
import cv2

cap = cv2.VideoCapture(0)
var = cap.get(11)
cap.set(11, 0.1) #default was .433, lower contrast is better
print var

while(1):
	#capture an image
	_, frame = cap.read() 
	#convert to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	#find green pixels image using limits in in numpy arrays
	lower_green = np.array([75,100,150]) 
	upper_green = np.array([95,255,255])

	#mask filters colors out of the green range from
	# the frame being read
	mask = cv2.inRange(hsv, lower_green, upper_green)
	#cv2.imshow('mask', mask)

	ret,thresh = cv2.threshold(mask,127,255,0)
	#cv2.imshow('thresh', thresh)

	# get contours in hopes that these can be converted to
	# rectangles	
	contours,hierarchy = cv2.findContours(thresh, 1, 2)
	if not len(contours) > 0:
		break
 
	cnt = contours[0]
	M = cv2.moments(cnt)
	print M
	print contours
	#approximate shape of object and draw a line surrounding the
	#object, make polygon a rectangle
	epsilon = 0.1*cv2.arcLength(cnt,True)
	approx = cv2.approxPolyDP(cnt,epsilon,True)
	cv2.imshow('approx', approx)
#ret is return value from the camera frame

#uses variables to speed up process, just shows process of contouring in terminal
	#perimeter = cv2.arcLength(cnt,True)
	#for (x,y,w,h) in tape:
		#cv2.rectangle(perimeter,(x,y),(x+w,y+h)(255,200,100),2)

	#result = cv2.bitwise_and(frame, frame, mask = mask)
#applies mask by saying input array 1 and 2 are frame, output array is mask (still don't fully understand why a bitwise_and is necessary)
	
	#cv2.imshow('frame', frame)
	#cv2.imshow('result', result)
#why can't you just show result? b/c when it doesn't work you want to know why...

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
#forgot how 0xFF works...I think it means something to do with wait for 15 characters or something... then quit, esc key probably closes windows
cv2.destroyAllWindows()
cap.release()


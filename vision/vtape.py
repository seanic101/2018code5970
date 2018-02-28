import numpy as np
import cv2
from location import Location
from math import sin, radians

cap = cv2.VideoCapture(0)

# Here are the numeric values of the properties that can be set on a VideoCapture:
# 0. CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
# 1. CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
# 3. CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file
# 4. CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
# 5. CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
# 6. CV_CAP_PROP_FPS Frame rate.
# 7. CV_CAP_PROP_FOURCC 4-character code of codec.
# 8. CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
# 9. CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
# 10. CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
# 11. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
# 12. CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
# 13. CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
# 14. *** CV_CAP_PROP_HUE Hue of the image (only for cameras).
# 15. *** CV_CAP_PROP_GAIN Gain of the image (only for cameras).
# 16. *** CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
# 17. CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
# 18. CV_CAP_PROP_WHITE_BALANCE Currently unsupported
# 19. CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
# *** not supported by MS camera


contrast = cap.get(11)
print("old contrast " + (str(contrast)))
#default was .433, lower contrast is better
new_con = cap.set(11, 0.1)
print("new contrast " + (str(new_con))) #-trast

# Setting of the camera exposure is not supported by this camera
#old_expo = cap.get(cv2.cv.CV_CAP_PROP_EXPOSURE)
#print "old exposure " + str(old_expo)
#new_expo = cap.set(cv2.cv.CV_CAP_PROP_EXPOSURE, .5)
#print new_expo

loc = Location()

FOV_x_deg = 58.0 # degrees
FOV_y_deg = 31.0 # degrees

FOV_x_pix = 640.0 # pixels
FOV_y_pix = 480.0 # pixels

Tape_W = 3.0 # inches
Tape_H = 15.3 # inches

def centerbox(box):
	center_x = box.x+box.w/2.0
	center_y = box.y+box.h/2.0
	#print(center_x)
	return (center_x, center_y) #pixels


def offset(center_x, center_y):
	return (
		center_x/FOV_x_pix,
		center_y/FOV_y_pix
	) # pure number - ratio 0-1 of screen until x or y

def distance(box, delta_x_deg, cen_x):
	half_FOV_x = FOV_x_pix / 2.0
	inches_per_pixel = Tape_W / box.w
	dis = (cen_x - half_FOV_x) / sin(radians(delta_x_deg)) #pixels
	return dis * inches_per_pixel

def where(box):
	cen_x, cen_y = centerbox(box)
	off_x, off_y = offset(cen_x, cen_y)
	delta_x_deg = off_x * FOV_x_deg - FOV_x_deg / 2.0
	return (
		delta_x_deg,
		off_y * FOV_y_deg - FOV_y_deg / 2.0,
		distance(box, delta_x_deg, cen_x)
	)	

class Box:
	def __init__(self, x, y, w, h):
		self.x = x
		self.y = y
		self.w = w
		self.h = h

import inspect
while(1):
	#capture an image
	_, frame = cap.read() 
	#convert to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	#find green pixels image using limits in in numpy arrays
	#lower_green = np.array([75,100,150]) 
	#upper_green = np.array([95,255,255])
	lower_green = np.array([75,100,150]) 
	upper_green = np.array([95,255,255])


	#mask filters colors out of the green range from
	# the frame being read
	mask = cv2.inRange(hsv, lower_green, upper_green)
	#cv2.imshow('mask', mask)

	#pixelates image, does not show small detections
	kernel = np.ones((5, 5), np.uint8)
	erosion = cv2.erode(mask, kernel, iterations=1)
	cv2.imshow('erosion', erosion)

	#contours the mask
	image,contours,hierarchy = cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	#finds largest object and contours it, saves in recordIndex
	recordSize = 0
	recordIndex = -1
	for i in range(len(contours)):
		if (cv2.contourArea(contours[i]) > recordSize):
			recordSize = cv2.contourArea(contours[i])
			recordIndex = i
	if recordIndex >= 0:
		#print "hello"
		#drawContours is destructive in OpenCV <3.x.x
		cv2.drawContours(hsv,contours,recordIndex,(0,255,0),3)
		#boundingRect output when printed is the (x,y and w,h)
		bound = cv2.boundingRect(contours[recordIndex])
		
	
		#print inspect.getmembers(bound)
		box = Box(*bound)
		print(where(box))

	cv2.imshow('hsv',hsv)

	#result = cv2.bitwise_and(frame, frame, mask = mask)
	
	#cv2.imshow('frame', frame)
	#cv2.imshow('result', result)

	#0xFF means to only look for last bit of the return value of
 	#waitKey so shift/alt etc... works
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()
cap.release()

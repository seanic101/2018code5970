import numpy as np
import cv2
from location import Location
from math import tan, radians

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


contrast = cap.get(cv2.cv.CV_CAP_PROP_CONTRAST)
print("old contrast " + (str(contrast)))
#default was .433, lower contrast is better
new_con = cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, 0.1)
print("new contrast " + (str(new_con))) #-trast

# Setting of the camera exposure is not supported by this camera
#old_expo = cap.get(cv2.cv.CV_CAP_PROP_EXPOSURE)
#print "old exposure " + str(old_expo)
#new_expo = cap.set(cv2.cv.CV_CAP_PROP_EXPOSURE, .5)
#print new_expo

loc = Location()

def centerbox(box):
	center_x = box.x()+box.w()/2
	center_y = box.y()+box.h()/2
	return (center_x, center_y)

FOV_x_deg = 58 # degrees
FOV_y_deg = 31 # degrees

FOV_x_pix = 640 # pixels
FOV_y_pix = 480 # pixels

Tape_W = 2.0 # inches
Tape_H = 15.3 # inches

def offset(center_x, center_y):
	return (
		center_x/FOV_x_pix * FOV_x_pix,
		center_y/FOV_y_pix * FOV_x_pix
	)

def degreesFOV(offset_x, offset_y):
	return (
		offset_x / FOV_x_pix * FOV_x_deg - FOV_x_deg / 2,
		offset_y / FOV_y_pix * FOV_y_deg - FOV_y_deg / 2
	)

def where(box):
	cen_x, cen_y = centerbox(box)
	off_x, off_y = offset(cen_x, cen_y)
	return (
		off_x * (FOV_x_deg / FOV_x_pix),
		off_y * (FOV_y_deg / FOV_y_pix),
		distance(box)
	)
	
def distance(box):
	return (
		Tape_W / box.w * (box.w / 2) * 
		math.tan(math.radians(FOV_x_deg / 2))
	)

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
	contours,hierarchy = cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

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
		#boundingRect output when printed is the (x,y and w,h)...maybe...pretty sure...
		bound = cv2.boundingRect(contours[recordIndex])
		print(bound)

	cv2.imshow('hsv',hsv)
	

	# get contours in hopes that these can be converted to
	# rectangles	
	#contours,hierarchy = cv2.findContours(thresh, 1, 2)
	#if not len(contours) > 0:
		#break
 
	#cnt = contours[0]
	#M = cv2.moments(cnt)
	#print M
	#print contours

#ret is return value from the camera frame

	#result = cv2.bitwise_and(frame, frame, mask = mask)
	
	#cv2.imshow('frame', frame)
	#cv2.imshow('result', result)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
#forgot how 0xFF works...I think it means something to do with wait for 15 characters or something... then quit, esc key closes windows
cv2.destroyAllWindows()
cap.release()

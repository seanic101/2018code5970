# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import numpy as np
import cv2
from location import Location
from math import sin, radians, sqrt
import time

DEBUG = True
if DEBUG:
	import inspect

cap = cv2.VideoCapture(0)
if not cap.isOpened():
	sys.stderr.write("No camera could be opened for capture\n")
	exit(1)

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

LOC = Location()

def setLocation(degrees, azim, distance):
	with MUTEX:
		LOC.degrees  = degrees
		LOC.azim     = azim
		LOC.distance = distance

FOV_x_deg = 58.0 # degrees
FOV_y_deg = 31.0 # degrees

FOV_x_pix = 640.0 # pixels
FOV_y_pix = 480.0 # pixels

#Tape_W = 3.0 # inches
#Tape_H = 15.3 # inches
#Big_W = 8.0 # inches: distance from outer edges of tape in x
Big_W = 5.1 # inches: distance from outer edges of tape in x
Tape_W = 1.5 # inches of camera box
Tape_H = 3.0 # inches of camera box

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

def my_distance(box, delta_x_deg, cen_x):
	half_FOV_x = FOV_x_pix / 2.0
	inches_per_pixel = Big_W / box.w
	tmp = sin(radians(delta_x_deg)) #pixels
	if tmp != 0.0:
		dis = (cen_x - half_FOV_x) / tmp * inches_per_pixel # inches
	else:
		dis = -1
	return dis

def where(box):
	cen_x, cen_y = centerbox(box)
	off_x, off_y = offset(cen_x, cen_y)
	delta_x_deg = off_x * FOV_x_deg - FOV_x_deg / 2.0
	return (
		delta_x_deg,
		off_y * FOV_y_deg - FOV_y_deg / 2.0,
		my_distance(box, delta_x_deg, cen_x)
	)	

class Box:
	def __init__(self, x, y, w, h):
		self.x = x
		self.y = y
		self.w = w
		self.h = h
	
	def __set_x(self, x):
		self.__x = x
	def __set_y(self, y):
		self.__y = y
	def __set_w(self, w):
		self.__w = w
	def __set_h(self, h):
		self.__h = h

	def __get_x(self):
		return self.__x
	def __get_y(self):
		return self.__y
	def __get_w(self):
		return self.__w
	def __get_h(self):
		return self.__h

	x = property(__set_x, __get_x)
	y = property(__set_y, __get_y)
	w = property(__set_w, __get_w)
	h = property(__set_h, __get_h)

	def bb(self):
		return (x, y, x+w, y+h)

def bb_of_two(boxA, boxB):
	x1A, y1A, x2A, y2A = boxA.bb()
	x1B, y1B, x2B, y2B = boxB.bb()
	ulx = min(x1A, x1B)
	uly = min(y1A, y1B)
	lrx = max(x2A, x2B)
	lry = max(y2A, y2B)
	w = lrx - ulx
	h = lry - uly
	return Box(ulx, uly, w, h)

def find_tape():
	while True:

		if not USING_TAPE:
			time.sleep(0.1)
			continue

		# capture an image
		retval, frame = cap.read() 
		if not retval:
			time.sleep(1)
			continue
		# convert to HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		# hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	
		# find green pixels image using limits in in numpy arrays
		# lower_green = np.array([75,100,150]) 
		# upper_green = np.array([95,255,255])
		lower_green = np.array([75,100,150]) 
		upper_green = np.array([95,255,255])
	
	
		# mask filters colors out of the green range from
		# the frame being read
		mask = cv2.inRange(hsv, lower_green, upper_green)
		# cv2.imshow('mask', mask)
	
		# pixelates image, does not show small detections
		kernel = np.ones((5, 5), np.uint8)
		erosion = cv2.erode(mask, kernel, iterations=1)
	
		if DEBUG:
			cv2.imshow('erosion', erosion)
	
		# contours the mask
		contours,hierarchy = cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	
		#image,contours,hierarchy = cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	
		# min_apect occurs when object is +/- 30 degrees off center
		min_aspect = Tape_W * sqrt(3.0) / 2.0 /15.3
		# max_apect occurs when object is directly in front of camera
		max_aspect = Tape_W / Tape_H
		
		# finds pairs of contours whose bounding rectangles have suitable
		# aspect ratios
		candidates = []
	
		for i in range(len(contours)):
			x,y,w,h = cv2.boundingRect(contours[i])
	
			if w < 10 or h < 10:
				continue
	
			aspect_ratio = float(w)/(h)
			if aspect_ratio < min_aspect or aspect_ratio > max_aspect:
				continue
	
			candidates.append({ "x":x, "y":y, "w":w, "h":h })
	
		if len(candidates) >= 2:
			candidates.sort(key=lambda o: o["h"], reverse=True)
			print "candidates " + str(candidates)
	
			for i in range(len(candidates)-1):
				# through out any candidates have poorly matching y values
				if abs(candidates[i]["y"] - candidates[i+1]["y"]) > (26):
					print("aaa")
					continue
	
				# through out any candidates have poorly matching h values
				del_h = abs(candidates[i]["h"] - candidates[i+1]["h"])
				if del_h > (candidates[i]["h"] * 0.1):
					print("bbb")
					continue
	
				print("XXXXXX")
	
				box = candidates[i]
				box1 = Box(
					 box["x"],            box["y"],
					 box["x"] + box["w"], box["y"] + box["h"])
				box = candidates[i+1]
				box2 = Box(
					 box["x"],            box["y"],
					 box["x"] + box["w"], box["y"] + box["h"])
	
				big_box = bb_of_two(box1, box2)
				degrees, azim, distance = where(big_box)
	
				# Set location of the big box
				setLocation(degrees, azim, distance)
	
				if DEBUG:
					box = candidates[i]
					cv2.rectangle(hsv,
						( box["x"],            box["y"]),
						( box["x"] + box["w"], box["y"] + box["h"]),
						(0,255,0),3)
					box = candidates[i+1]
					cv2.rectangle(hsv,
						( box["x"],            box["y"]),
						( box["x"] + box["w"], box["y"] + box["h"]),
						(0,255,0),3)
					cv2.imshow('hsv',hsv)
	
						# drawContours is destructive in OpenCV <3.x.x
						#cv2.drawContours(hsv,candidates,i,(0,255,0),3)
						#cv2.drawContours(hsv,candidates,i+1,(0,255,0),3)
						#cv2.drawContours(hsv,contours,1,(0,255,0),3)
			
					# boundingRect output when printed is the (x,y and w,h)
					#bound0= cv2.boundingRect(contours[candidates[i]])
					#bound1= cv2.boundingRect(contours[candidates[i+1]])
	
			#print "bound0 " + str(bound0)
	
			#box = Box(*bound0)
			#print(where(box))
			#box = Box(*bound1)
			#print(where(box))
	
	
		#result = cv2.bitwise_and(frame, frame, mask = mask)
		
		#cv2.imshow('frame', frame)
		#cv2.imshow('result', result)
	
		# 0xFF means to only look for last bit of the return value of
	 	# waitKey so shift/alt etc... works
		k = cv2.waitKey(5) & 0xFF
		if k == 27:
			break

if DEBUG:	
	cv2.destroyAllWindows()
cap.release()

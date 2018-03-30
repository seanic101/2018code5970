#! /usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import numpy as np
import cv2
from location import Location
from math import sin, radians, sqrt
import time
from camera import Camera
import mmap
import json
import logging

def setLocation(mm, mutex, degrees, azim, distance):
	loc = Location()
	loc.degrees  = degrees
	loc.azim     = azim
	loc.distance = distance
	json_data = (
		json.dumps(
		(loc.degrees, loc.azim, loc.distance),
		ensure_ascii=False) + '\n'
		)
	with mutex:
		mm.seek(0)
		mm.write(json_data)

	logging.debug("degrees: " + str(loc.degrees) + " azimuth: " + str(loc.azim) + " distance: " + str(loc.distance))

FOV_x_deg = 58.0 # degrees
FOV_y_deg = 31.0 # degrees

FOV_x_pix = 640.0 # pixels
FOV_y_pix = 480.0 # pixels

#Tape_W = 3.0 # inches
#Tape_H = 15.3 # inches
#Big_W = 8.0 # inches: distance from outer edges of tape in x
Big_W = 4.02 # inches: distance from outer edges of tape in x
Tape_W = 1.18 # inches of camera box
Tape_H = 2.08 # inches of camera box

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
		#FUDGE FACTOR
		dis *= 1.1
	else:
		dis = -1
	return dis

def where(box):
	cen_x, cen_y = centerbox(box)
	off_x, off_y = offset(cen_x, cen_y)
	#off_x *= -1
	#off_y *= -1
	delta_x_deg = off_x * FOV_x_deg - FOV_x_deg / 2.0
	delta_y_deg = -1.0 * (off_y * FOV_y_deg - FOV_y_deg / 2.0)
	#print(off_x, off_y)
	#print(cen_x, cen_y)	
	
	return (
		delta_x_deg,
		delta_y_deg,
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
		return (self.x, self.y, self.x + self.w, self.y + self.h)

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

def find_tape(mm, mutex, cam):
	global DEBUG
	logging.debug("cam " + str(cam))
	while True:
		# capture an image
		retval, frame = cam.read() 
		if not retval:
			time.sleep(1)
			continue

		logging.debug("got image from cam")

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
		contours,hierarchy = (
			cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
			)

		logging.debug("got contours")
	
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
			logging.debug("candidates " + str(candidates))
	
			for i in range(len(candidates)-1):
				# through out any candidates have poorly matching y values
				if abs(candidates[i]["y"] - candidates[i+1]["y"]) > (26):
					if DEBUG:
						print("Difference of y values is too large")
					continue
	
				# through out any candidates have poorly matching h values
				del_h = abs(candidates[i]["h"] - candidates[i+1]["h"])
				if del_h > (candidates[i]["h"] * 0.1):
					if DEBUG:
						print("Difference of height is too large")
					continue

				logging.info("Passed filters")
	
				box = candidates[i]
				box1 = Box(box["x"], box["y"], box["w"], box["h"])
				box = candidates[i+1]
				box2 = Box(box["x"], box["y"], box["w"], box["h"])
	
				big_box = bb_of_two(box1, box2)
				degrees, azim, distance = where(big_box)
	
				#print(degrees, azim, distance)
				# Set location of the big box
				setLocation(mm, mutex, degrees, azim, distance)
	
				if DEBUG:
					cv2.rectangle(hsv,
						(box1.x, box1.y), (box1.x + box1.w, box1.y + box1.h),
						(0,255,0),3)
					cv2.rectangle(hsv,
						(box2.x, box2.y), (box2.x + box2.w, box2.y + box2.h),
						(0,255,0),3)
					cv2.imshow('hsv',hsv)
	
		# 0xFF means to only look for last bit of the return value of
	 	# waitKey so shift/alt etc... works
		k = cv2.waitKey(5) & 0xFF
		if k == 27:
			break

DEBUG = False
if __name__ == "__main__":
	from multiprocessing import Lock
	import inspect
	DEBUG = True
	logging.basicConfig(filename='logger.log', level=logging.DEBUG)
	logging.debug("start")
	cam = Camera(0)
	cam.set_debug(True)
	cam.setup()
	# Build a mutex to protect a location instance
	mutex = Lock()

	s = (
		b"01234567890123456789012345678901234567890123456789" +
		b"01234567890123456789012345678901234567890123456789"
		)
	mm = mmap.mmap(-1, 100)
	mm.write(s)

	find_tape(mm, mutex, cam)
	cam.shutdown()
	mm.close()

	
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


# Setting of the camera exposure is not supported by this camera
#old_expo = cap.get(cv2.cv.CV_CAP_PROP_EXPOSURE)
#print "old exposure " + str(old_expo)
#new_expo = cap.set(cv2.cv.CV_CAP_PROP_EXPOSURE, .5)
#print new_expo
	




import numpy as np
import cv2


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


#contrast = cap.get(cv2.cv.CV_CAP_PROP_CONTRAST)
#print "old contrast " + str(contrast)
#default was .433, lower contrast is better
#new_con = cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, 0.01)
#print "new contrast " + str(new_con) #-trast

# Setting of the camera exposure is not supported by this camera
#old_expo = cap.get(cv2.cv.CV_CAP_PROP_EXPOSURE)
#print "old exposure " + str(old_expo)
#new_expo = cap.set(cv2.cv.CV_CAP_PROP_EXPOSURE, .5)
#print new_expo

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
	kernel = np.ones((3, 3), np.uint8)
	erosion = cv2.erode(mask, kernel, iterations=1)
	cv2.imshow('erosion', erosion)

	#contours the mask, erosion variable below possibly causing
	#errors
	contours,hierarchy = cv2.findContours(erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	#contours = []

	#finds largest object and contours it, saves in recordIndex
	recordSize = 0
	recordIndex = -1
	for i in range(len(contours)):
		if (cv2.contourArea(contours[i]) > recordSize):
			recordSize = cv2.contourArea(contours[i])
			recordIndex = i
	if recordIndex >= 0:
		#print "hello"
		cv2.drawContours(hsv,contours,recordIndex,(0,255,0),3)
	cv2.imshow('hsv',hsv)

	#ret,thresh = cv2.threshold(mask,127,255,0)
	#cv2.imshow('thresh', thresh)

	# get contours in hopes that these can be converted to
	# rectangles	
	#contours,hierarchy = cv2.findContours(thresh, 1, 2)
	#if not len(contours) > 0:
		#break
 
	#cnt = contours[0]
	#M = cv2.moments(cnt)
	#print M
	#print contours

	#approximate shape of object and draw a line surrounding the
	#object, make polygon a rectangle
	#epsilon = 0.1*cv2.arcLength(cnt,True)
	#approx = cv2.approxPolyDP(cnt,epsilon,True)
	#cv2.imshow('approx', approx)

#ret is return value from the camera frame

#uses variables to speed up process, just shows process of contouring in terminal
	#perimeter = cv2.arcLength(cnt,True)
	#for (x,y,w,h) in tape:
		#cv2.rectangle(perimeter,(x,y),(x+w,y+h)(255,200,100),2)

	#result = cv2.bitwise_and(frame, frame, mask = mask)
	
	#cv2.imshow('frame', frame)
	#cv2.imshow('result', result)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
#forgot how 0xFF works...I think it means something to do with wait for 15 characters or something... then quit, esc key closes windows
cv2.destroyAllWindows()
cap.release()

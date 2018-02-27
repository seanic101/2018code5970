import cv2
import numpy

powercube_cascade = cv2.CascadeClassifier('powercube_cascade.xml')

cap = cv2.VideoCapture(0)

while 1:
	ret, img = cap.read() #reads captured image?
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

	cubes = powercube_cascade.detectMultiScale(gray, 1.3, 5) #turns faces gray
	#watches = watch_cascade.detectMultiScale(gray, 50, 50) #turns watches 50x50 pixels gray
	for (x,y,w,h) in cubes: #for each dimension of the watch
		print "gosh darnit why won't this work"
		cv2.rectangle(img,(x,y),(x+w, y+h),(255,0,0),2) 

	cv2.imshow('img',img) #displays rectangles on vid
	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break

cap.release()
cap.destroyAllWindows()

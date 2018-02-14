import cv2
import numpy

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
watch_cascade = cv2.CascadeClassifier('watchcascade10stage.xml')

cap = cv2.VideoCapture(0)

while 1:
	ret, img = cap.read() #reads captured image?
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
#gray is the grayed version of image
	faces = face_cascade.detectMultiScale(gray, 1.3, 5) #turns faces gray
	watches = watch_cascade.detectMultiScale(gray, 50, 50) #turns watches 50x50 pixels gray
	for (x,y,w,h) in watches: #for each dimension of the watch
		print "gosh darnit why won't this work"
		cv2.rectangle(img,(x,y),(x+w, y+h),(255,0,0),2) 
#draw rectangle at x,y with x+w side length, y+h height. with 255 red?

	cv2.rectangle(img,(0,0),(50,30),(255,255,255),2)
	for (x,y,w,h) in faces:
		print "hi"
		cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
#how does this know to draw rectangle around the face itself? Does this somehow reference face_cascade?
	
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = img[y:y+h, x:x+w]
		eyes = eye_cascade.detectMultiScale(roi_gray) 
#tells to find eyes within the face

		for (ex,ey,ew,eh) in eyes:
			cv2.rectangle(img,(x,y),(ex+ew,ey+eh),(0,255,1),2)
#what does the '...,2)' do?
	cv2.imshow('img',img) #displays rectangles on vid
	k = cv2.waitKey(30) & 0xff
	if k == 27:
#this function waits for a key which indicates to close/stop program (escape)
		break

cap.release()
cap.destroyAllWindows()














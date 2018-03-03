import cv2
import numpy

powercube_cascade = cv2.CascadeClassifier('powercube_cascade.xml')

cap = cv2.VideoCapture(0)


#testing
FOV_x_deg = 58.0 # degrees
FOV_y_deg = 31.0 # degrees

FOV_x_pix = 640.0 # pixels
FOV_y_pix = 480.0 # pixels

#Tape_W = 3.0 # inches
#Tape_H = 15.3 # inches
Tape_W = 3.0 # inches of camera box
Tape_H = 6.0 # inches of camera box

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




#working
while 1:
	ret, img = cap.read()
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

	cubes = powercube_cascade.detectMultiScale(gray, 1.3, 5)
	#cubes = powercube_cascade.detectMultiScale(gray, 50, 50)

	for (x,y,w,h) in cubes:
		#print("gosh darnit why won't this work")
		box = cv2.rectangle(img,(x,y),(x+w, y+h),(255,0,0),2)
		#print(box)


	cv2.imshow('img',img)
	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break

cap.release()
cv2.destroyAllWindows()

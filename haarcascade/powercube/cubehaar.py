import urllib
import numpy as np
import cv2
import os

if not os.path.exists('pos'):
	os.makedirs('small')
pic_num=1
#imgpath = '~/opencv_workspace/robotgit/2018code5970/haarcascade/powercube/pos/'+str(pic_num)+'.jpg'
#imgpath = ('/home/opencv_workspace/robotgit/2018code5970/haarcascade/powercube/pos/')
imgpath = ('pos/')

for pic in os.listdir('pos/'):
	img_name = imgpath+pic	
	print img_name
	print "hi"
	img = cv2.imread(img_name,cv2.IMREAD_GRAYSCALE)
	print img
	resized_image = cv2.resize(img, (50,50))
	print "hello"
	cv2.imwrite(imgpath+str(pic_num) + '.jpg',resized_image)
	print "snort"
	pic_num += 1

		#img = cv2.imread(negnumjpg, cv2.IMREAD_GRAYSCALE) #makes pic gray
		#resized_image = cv2.resize(img, (100, 100)) #makes pic 100x100 pixels
		#cv2.imwrite(negnumjpg, resized_image) #overwrites old garbo image,

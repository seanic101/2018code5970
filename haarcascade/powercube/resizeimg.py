import urllib
import numpy as np
import cv2
import os

if not os.path.exists('pos'):
	os.makedirs('pos')
pic_num=1
imgpath = 'sourceimg/'

for pic in os.listdir('sourceimg/'):
	img_name = imgpath+pic
	print img_name
	print "hi"
	img = cv2.imread(img_name,cv2.IMREAD_GRAYSCALE)
	print img
	resized_image = cv2.resize(img, (160,160))
	print "hello"
	cv2.imwrite('pos/'+str(pic_num) + '.jpg',resized_image)
	print "snort"
	pic_num += 1

		#img = cv2.imread(negnumjpg, cv2.IMREAD_GRAYSCALE)
		#resized_image = cv2.resize(img, (100, 100)) 
		#cv2.imwrite(negnumjpg, resized_image)

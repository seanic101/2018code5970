import urllib
import numpy as np
import cv2
import os

#pos = '~/opencv_workspace/robotgit/2018code5970/haarcascade/pos/'
pos_num = 1
for pic in "pos/":
	img = cv2.imread("pos/"+str(pos_num)+".jpg", cv2.IMREAD_GRAYSCALE)
	resizedimage = cv2.resize(img, (50, 50))
	cv2.imwrite("pos/"+str(pos_num)+".jpg",resizedimage)
	pos_num += 1

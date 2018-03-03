import cv2
import os
import re

vidcap = cv2.VideoCapture('IMG_0708.mov')
success,image = vidcap.read()
count = 0

def vidlen(vidcap):
	
success = True
while success:
	success,image = vidcap.read()
	if not success:
		break
	if (count % 2) == 0:
		print('Writing a new frame: ', success)
		cv2.imwrite("frame%04d.jpg" % count, image) # save frame as JPEG file
	count += 1
	#if count >= 2000:
		#break

imgpath = './'
bg_txt_lines = ''
ls = os.listdir(imgpath)
ls.sort()
for pic in ls:
	if not re.match(r'.*\.jpg', pic):
		print("skipping " + pic)
		continue

	img_name = imgpath + pic	
	print img_name
	img = cv2.imread(img_name, cv2.IMREAD_GRAYSCALE)
	resized_image = cv2.resize(img, (160,160))
	rows,cols = resized_image.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
	dst = cv2.warpAffine(resized_image,M,(cols,rows))
	
	cv2.imwrite(img_name, dst)
	bg_txt_lines += 'neg/' + pic + '\n'

with open('bg.txt', 'w') as f:
	f.write(bg_txt_lines)


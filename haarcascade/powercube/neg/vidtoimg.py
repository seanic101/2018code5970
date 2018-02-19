import cv2

vidcap = cv2.VideoCapture('')
success,image = vidcap.read()
count = 0
success = True
while success:
	success,image = vidcap.read()
	print('Read a new frame: ', success)
	if not success:
		break
	cv2.imwrite("frame%d.jpg" % count, image)     # save frame as JPEG file
	count += 1
	if count >= 1500:
		break



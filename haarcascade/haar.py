import urllib #different requests/retrieves for py 3
import cv2
import numpy as np
import os

def store_raw_images():
	# 1, inactive, done http://image-net.org/api/text/imagenet.synset.geturls?wnid=n07942152
	# 2, active http://image-net.org/api/text/imagenet.synset.geturls?wnid=n00523513
	neg_images_link = 'http://image-net.org/api/text/imagenet.synset.geturls?wnid=n00523513'
	neg_image_urls = urllib.urlopen(neg_images_link).read().decode()
	pic_num = 1397 #update pic_num with (last number of link 1) + 1 when running second link

	if not os.path.exists('neg'):
		os.makedirs('neg') #makes dir desired

	for link in neg_image_urls.split('\n'):
		#each url ends in 'Enter', '\n' equals 'Enter', thus we can split each URL up, using loop to read each
		try:
			print(link)
			negnumjpg = "neg/"+str(pic_num)+".jpg"
			urllib.urlretrieve(link, negnumjpg) #opens links provided above
			img = cv2.imread(negnumjpg, cv2.IMREAD_GRAYSCALE) #makes pic gray
			resized_image = cv2.resize(img, (100, 100)) #makes pic 100x100 pixels
			cv2.imwrite(negnumjpg, resized_image) #overwrites old garbo image, saves new image
			pic_num += 1

		except Exception as e:
			print str(e)

def find_uglies():
	match = False
	for file_type in ['neg']:
		for img in os.listdir(file_type):#why is this loop necessary?
			for ugly in os.listdir('uglies'):
				try:
					current_image_path = str(file_type)+'/'+str(img)
#what does the str(file_type) do? wouldn't it find the same as str(img)?
					ugly = cv2.imread('uglies/'+str(ugly))
					question = cv2.imread(current_image_path)
					if ugly.shape == question.shape and \
					not(np.bitwise_xor(ugly,question).any()):
 #what is bitwise and why is this np.bitwise part necessary?
						print str('Gross. Picture does not work, will delete now.')
						print current_image_path
						os.remove(current_image_path)
				except Exception as e:
					print str(e)
#store_raw_images()
#find_uglies()

def create_pos_n_neg():
	for file_type in ['neg']:
		for img in os.listdir(file_type): #why is this second loop necessary?
			if file_type == 'pos':
				line = file_type+'/'+img+' 1 0 0 50 50\n'
# ' 1 0 0 50 50\n' means 1 object in image, location of rectangle in image for 50 by 50 pixels
			elif file_type == 'neg': #shouldn't they all be neg and this always true?
				line = file_type+'/'+img+'\n'
				with open('bg.txt', 'a') as f:
#what does previous line do? (59)
					f.write(line)
#create_pos_n_neg()	############################################################################					


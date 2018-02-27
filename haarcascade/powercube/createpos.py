import os
import re

imgpath = 'resizedpos'
ls = os.listdir(imgpath)
ls.sort()
for pic in ls:
	
	if not re.match(r'.*\.jpg', pic):
		print("skipping " + pic)
		continue

	os.system('opencv_createsamples -bg bg.txt -info info/info.lst'+
		' -pngoutput info -maxxangle 0.5 -maxyangle 0.5'+
		' -maxzangle 0.5 -num 237 -img pos/' + pic)
#changed .dat to .lst
#will have to do math for the specific num of samples
#script must be run from dir containing positive images intended to become models for combined positives and negatives which will go in the vector file

import os
import re

imgpath = 'pos'
ls = os.listdir(imgpath)
ls.sort()
for pic in ls:
	
	if not re.match(r'.*\.jpg', pic):
		print("skipping " + pic)
		continue

	os.system('opencv_createsamples -img ' + pic + ' -bg bg.txt -info info/info.lst -pngoutput info -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -num 237')
 
#will have to do math for the num of samples

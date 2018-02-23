import os
import re

imgpath = 'pos'
ls = os.listdir(imgpath)
ls.sort()
for pic in ls:
	
	if not re.match(r'.*\.jpg', pic):
		print("skipping " + pic)
		continue

	#os.system('opencv_createsamples -bg bg.txt -info info/info.dat -pngoutput info -maxxangle 0.5 -maxyangle 0.5 -maxzangle 0.5 -num 237 -vec ' + pic + '.vec')

	os.system('opencv_createsamples -bg bg.txt -info info/info.lst'+
		' -pngoutput info -maxxangle 0.5 -maxyangle 0.5'+
		' -maxzangle 0.5 -num 237 -img pos/' + pic)
#changed .dat to .lst
#will have to do math for the num of samples

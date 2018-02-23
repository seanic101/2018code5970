
import os
import re

imgpath = 'posandneg'
ls = os.listdir(imgpath)
ls.sort()
for pic in ls:
	
	#if not re.match(r'.*\.jpg', pic):
		#print("skipping " + pic)
		#continue

	line = imgpath + '/'+ pic +' 1 0 0 160 160' +'\n'
	with open('info.lst','a') as f:
		f.write(line)

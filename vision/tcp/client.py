#!/usr/bin/python

import sys
sys.path.insert(0, '/home/nvidia/opencv_workspace/robotgit/2018code5970/vision')
import socket
import json
from time import sleep
from server import parse
import re

MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)
LOC_DEFAULT = "locate_tape:" + json.dumps({}, ensure_ascii=False)
DEBUG_DEFAULT = "debug_on:" + json.dumps({'filename':'/tmp/debugout'}, ensure_ascii=False)

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
#s.send(DEBUG_DEFAULT)

try:
	while 1:
		sleep(0.1)
		s.send(LOC_DEFAULT)
		cmd, json_data = parse(s.recv(BUFFER_SIZE))
		print("client received loc data: <"+ json_data + ">")
#KeyboardInterrupt is Ctrl-C
except KeyboardInterrupt:
	print('interrupted')

s.send(MSG_DEFAULT)
data = s.recv(BUFFER_SIZE)
#print(data)
s.close()

print("client received shutdown data:" + str(data))

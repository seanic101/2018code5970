#! /usr/bin/env python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import sys
#Jetson path
sys.path.insert(0, '/home/nvidia/opencv_workspace/robotgit/2018code5970/vision')
#Windows path
sys.path.insert(0,"C:/Users/Beavertronics/Desktop/2018Workstation/2018code5970/vision/tcp")
import socket
import json
from time import sleep
from server import parse
import re
import sys
import argparse

PY2 = sys.version_info[0] == 2

MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)
LOC_DEFAULT = "locate_tape:" + json.dumps({}, ensure_ascii=False)
DEBUG_DEFAULT = "debug_on:" + json.dumps({'filename':'/tmp/debugout'}, ensure_ascii=False)

TCP_IP = '10.59.70.12'
TCP_PORT = 5005
BUFFER_SIZE = 1024

parser = argparse.ArgumentParser(description="Beavertronics Jetson TX1 client")

parser.add_argument('-d', '--debug', action='store_true')

args = parser.parse_args()
if args.debug:
	print("Connecting to server on localhost...")
	TCP_IP = '127.0.0.1'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

try:
	while 1:
		sleep(0.1)
		if PY2:
			s.send(LOC_DEFAULT)
		else:
			s.send(bytes(LOC_DEFAULT, 'utf-8'))

		cmd, json_data = parse(s.recv(BUFFER_SIZE))
		if PY2:
			tmp =  json_data
		else:
			tmp =  bytes_decode(json_data)
			
		print("client received loc data: <"+ tmp + ">")
#KeyboardInterrupt is Ctrl-C
except KeyboardInterrupt:
	print('interrupted')

s.send(MSG_DEFAULT)
json_data = s.recv(BUFFER_SIZE)
s.close()

if PY2:
	tmp =  json_data
else:
	tmp =  bytes_decode(json_data)
print("client received shutdown data:" + tmp)

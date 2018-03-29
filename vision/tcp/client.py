#! /usr/bin/env python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import os
import sys

#Jetson path
here = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, here + '/..')

#Windows path
#sys.path.insert(0,"C:/Users/Beavertronics/Desktop/2018Workstation/2018code5970/vision/tcp")
import socket
import json
from time import sleep
from server import parse, decode_json
import re
import argparse

PY2 = sys.version_info[0] == 2

MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)
LOC_DEFAULT = "locate_tape:" + json.dumps({}, ensure_ascii=False)
RESET_DEFAULT = "reset_tape:"  + json.dumps({}, ensure_ascii=False)
DEBUG_ON_DEFAULT = "debug_on:"  + json.dumps({}, ensure_ascii=False)
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

if args.debug:
	print("Connection to local host established")

try:

	if PY2:
		s.send(RESET_DEFAULT)
	else:
		s.send(bytes(RESET_DEFAULT, 'utf-8'))

	cmd, json_data = parse(s.recv(BUFFER_SIZE))
	if args.debug:
		print("Got json_data: <" + str(json_data) + ">")

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
			
		degrees, azim, distance = decode_json(tmp)
		print("client received loc data: <"+ str(degrees) + " " + str(azim) + " " + str(distance) + ">")
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

#! /usr/bin/env python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4
#
# Shutdown the OpenCV Vision system

import os
import sys

#Jetson path
here = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, here + '/tcp')

import socket
import json
from time import sleep
from server import parse, decode_json
import re
import argparse

PY2 = sys.version_info[0] == 2

MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)

TCP_IP = '10.59.70.12'
TCP_PORT = 5005
BUFFER_SIZE = 1024

parser = argparse.ArgumentParser(description="Beavertronics Jetson TX1 shutdown")

parser.add_argument('-d', '--debug', action='store_true')

args = parser.parse_args()
if args.debug:
	print("Connecting to server on localhost...")
	TCP_IP = '127.0.0.1'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

print("Connection to local host established")

s.send(MSG_DEFAULT)
json_data = s.recv(BUFFER_SIZE)
s.shutdown(socket.SHUT_WR)
dummy = s.recv(BUFFER_SIZE)
s.close()

if PY2:
	tmp =  json_data
else:
	tmp =  bytes_decode(json_data)
print("client received shutdown data:" + tmp)

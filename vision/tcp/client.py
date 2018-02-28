#!/usr/bin/python

import socket
import json

MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(MSG_DEFAULT)
data = s.recv(BUFFER_SIZE)
s.close()

print "client received data:", data

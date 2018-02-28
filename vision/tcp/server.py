#!/usr/bin/python

import socket
import sys
sys.path.insert(0, '..')
#import os
import re
import json
from location import Location

RSP_DEFAULT = "Success:" + json.dumps({}, ensure_ascii=False)
POWERCUBUE_LOCATION = Location()
TAPE_LOCATION = Location()
DEBUG = False

def init_debug(filename):
	if not DEBUG:
		DEBUG_LOG = open(filename, 'w')
		# Setting DEBUG True or False changes if we want to debug
		DEBUG = True

def close_debug():
	if DEBUG:
		DEBUG_LOG.close()
		DEBUG = False

def debug_out(s):
	if DEBUG:
		DEBUG_LOG.write("Debug: " + s)

def stderrout(s):
	sys.stderr.write("Error: " + s)

# parse matches patterns of the data into groups 1 and 2 separated
# by a colon and returns them separately
def parse(data):
	pattern = '([^:]+):(.*)'
	m = re.match(pattern, data)
	cmd = m.group(1)
	json_data = m.group(2)
	return (cmd, json_data)

def decode_json(json_data):
	decoded = ''
	try:
		decoded = json.loads(json_data)
 
	except (ValueError, KeyError, TypeError):
		stderrout("JSON format error")
	return decoded

def reset_powercube():
	POWERCUBE_LOCATION.reset()
	return RSP_DEFAULT

def reset_tape():
	TAPE_LOCATION.reset()
	return RSP_DEFAULT

def locate_powercube():
	# XXX phoney location until we serialize Location class
	return "Success:" + (
		json.dumps({[45.0, 10.0, 100]}, ensure_ascii=False)
	)

def locate_tape():
	# XXX phoney location until we serialize Location class
	return "Success:" + (
		json.dumps({[45.0, 10.0, 100]}, ensure_ascii=False)
	)

def shutdown():
	conn.send("Shutting down...")
	conn.close()
	sys.exit(0)
	return RSP_DEFAULT

def debug_on(decoded):
	# Access data
	filename = decoded['filename']
	init_debug(filename)
	return RSP_DEFAULT

def debug_off():
	close_debug()
	return RSP_DEFAULT

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
debug_out('Connection address: <' + str(addr) + '>')
while 1:
	data = conn.recv(BUFFER_SIZE)
	if not data:
		stderrout("receiving data")
		continue

	debug_out("received data: <" + data + '>')

	# parse the command out of data, return cmd and json args
	cmd, json_data = parse(data)
	decoded = decode_json(json_data)
	
	# route command to the handler method
	if cmd == 'reset_powercube':
		rsp = reset_powercube()

	elif cmd == 'reset_tape':
		rsp = reset_tape()

	elif cmd == 'locate_powercube':
		rsp = locate_powercube()

	elif cmd == 'locate_tape':
		rsp = locate_tape()

	elif cmd == 'shutdown':
		rsp = shutdown()

	elif cmd == 'debug_on':
		rsp = debug_on(decoded)

	elif cmd == 'debug_off':
		rsp = debug_off()

	else:
		s = "Not a command: " + cmd
		stderrout(s)
		rsp = "Error:" + s

	# send handler response back
	conn.send(rsp)



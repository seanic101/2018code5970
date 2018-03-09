#!/usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import socket
import sys
import re
import json
from location import Location

RSP_DEFAULT = "Success:" + json.dumps({}, ensure_ascii=False)
#POWERCUBUE_LOCATION = Location()
TAPE_LOCATION = None
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

DEBUG = False

import signal
import time

# Handle TERM and INT signals so we can close the network
# connection cleanly
class GracefulKiller:
	kill_now = False
	def __init__(self):
		signal.signal(signal.SIGINT, self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)

	def exit_gracefully(self,signum, frame):
		self.kill_now = True

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
		json.dumps((loc.degrees, loc.azim, loc.distance), ensure_ascii=False)
	)

def locate_tape():
	return "Success:" + (
		json.dumps((loc.degrees, loc.azim, loc.distance), ensure_ascii=False)
	)

def shutdown():
	return RSP_DEFAULT

def debug_on(decoded):
	# Access data
	filename = decoded['filename']
	init_debug(filename)
	return RSP_DEFAULT

def debug_off():
	close_debug()
	return RSP_DEFAULT

def jetson_server(loc, tcp_ip_address, tcp_port):
	TAPE_LOCATION = loc
	my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	my_socket.bind((tcp_ip_address, tcp_port))
	my_socket.listen(1)
	
	my_conn, my_addr = my_socket.accept()
	debug_out('Connection address: <' + str(my_addr) + '>')
	running = True
	while running:
		data = my_conn.recv(BUFFER_SIZE)
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
			USING_TAPE = False
			USING_CUBE = True
			rsp = locate_powercube()
	
		elif cmd == 'locate_tape':
			USING_TAPE = True
			USING_CUBE = False
			rsp = locate_tape()
	
		elif cmd == 'shutdown':
			running = False
			rsp = shutdown()
	
		elif cmd == 'debug_on':
			rsp = debug_on(decoded)
	
		elif cmd == 'debug_off':
			rsp = debug_off()
	
		else:
			s = "Not a command: " + cmd
			stderrout(s)
			rsp = "Error:" + s
	
		if killer.kill_now:
			break

		# send handler response back
		my_conn.send(rsp)

	my_conn.close()
	sys.exit(0)

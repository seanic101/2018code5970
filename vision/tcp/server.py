#!/usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4

import socket
import sys
PY2 = sys.version_info[0] == 2

import re
import json
from location import Location
import logging

RSP_DEFAULT = "Success:" + json.dumps({}, ensure_ascii=False)
#POWERCUBUE_LOCATION = Location()
#TAPE_LOCATION = None
BUFFER_SIZE = 100  # Normally 1024, but we want fast response

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

DEBUG_LOG = None

def init_debug(filename):
	#global DEBUG
	global DEBUG_LOG
	if not DEBUG_LOG:
		DEBUG_LOG = open(filename, 'w')
		# Setting DEBUG True or False changes if we want to debug
		DEBUG = True

def close_debug():
	#global DEBUG
	global DEBUG_LOG
	if DEBUG_LOG:
		DEBUG_LOG.close()
		DEBUG = False

def debug_out(s):
	global DEBUG_LOG
	#global DEBUG
	if DEBUG_LOG:
		DEBUG_LOG.write("Debug: " + str(s))

def stderrout(s):
	sys.stderr.write("Error: " + str(s))

# parse matches patterns of the data into groups 1 and 2 separated
# by a colon and returns them separately
def parse(data):
	global DEBUG_LOG
	global PY2

	if PY2:
	    pattern = '([^:]+):(.*)'
	else:
	    pattern = b'([^:]+):(.*)'

	if DEBUG_LOG:
		DEBUG_LOG.write("parsing cmd from client " + str(data) + "\n")
	m = re.match(pattern, data)
	if m == None:
		cmd = None
		json_data = None
	else:
		cmd = m.group(1)
		json_data = m.group(2)

	return (cmd, json_data)

def decode_json(json_data):
	decoded = ''
	try:
		decoded = json.loads(json_data)
 
	except (ValueError, KeyError, TypeError):
		stderrout("JSON format error <" + json_data + ">\n")
	return decoded

def reset_powercube():
	global RSP_DEFAULT
	POWERCUBE_LOCATION.reset()
	return RSP_DEFAULT

def reset_tape(mm, mutex):
	global RSP_DEFAULT
	loc = Location()
	json_data = json.dumps(
		(loc.degrees, loc.azim, loc.distance),
		ensure_ascii=False) + '\n'
	with mutex:
		mm.seek(0)
		mm.write(json_data)
	return RSP_DEFAULT

def locate_powercube(mm, mutex):
	# XXX phoney location until we serialize Location class
	with mutex:
		mm.seek(0)
		json_data = mm.readline()
	return "Success:" + json_data.rstrip('\n')
	#return "Success:" + (
	#	json.dumps((loc.degrees, loc.azim, loc.distance), ensure_ascii=False)
	#)

def locate_tape(mm, mutex):
	with mutex:
		mm.seek(0)
		json_data = mm.readline()
	logging.debug("locate_tape: json_data <" + json_data + ">")
	return "Success:" + json_data.rstrip('\n')
	#return "Success:" + (
	#	json.dumps((loc.degrees, loc.azim, loc.distance), ensure_ascii=False)
	#)

def shutdown():
	return RSP_DEFAULT

def debug_on(decoded):
	global RSP_DEFAULT
	# Access data
	filename = decoded['filename']
	init_debug(filename)
	return RSP_DEFAULT

def debug_off():
	global RSP_DEFAULT
	close_debug()
	return RSP_DEFAULT

def jetson_server(mm, mutex, tcp_ip_address, tcp_port):
	global BUFFER_SIZE
	loc = Location()
	my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sys.stderr.write("tcp_ip_address " + str(tcp_ip_address))
	my_socket.bind((tcp_ip_address, tcp_port))
	my_socket.listen(1)
	
	my_conn, my_addr = my_socket.accept()
	debug_out('Connection address: <' + str(my_addr) + '>')
	killer = GracefulKiller()
	running = True
	while running:
		data = my_conn.recv(BUFFER_SIZE)
		if not data:
			stderrout("No data received\n")
			continue
	
		#debug_out('received data: <' + data + '>\n')
	
		# parse the command out of data, return cmd and json args
		cmd, json_data = parse(data)
		decoded = decode_json(json_data)
		
		# route command to the handler method
		if cmd == 'reset_powercube':
			rsp = reset_powercube()
	
		elif cmd == 'reset_tape':
			rsp = reset_tape(mm, mutex)
	
		elif cmd == 'locate_powercube':
			rsp = locate_powercube()
	
		elif cmd == 'locate_tape':
			rsp = locate_tape(mm, mutex)
	
		elif cmd == 'shutdown':
			running = False
			rsp = shutdown()
	
		elif cmd == 'debug_on':
			rsp = debug_on(decoded)
	
		elif cmd == 'debug_off':
			rsp = debug_off()
	
		else:
			s = 'Not a command: <' + cmd + '>\n'
			stderrout(s)
			rsp = 'Error:' + s
	
		if killer.kill_now:
			break

		# send handler response back
		my_conn.send(rsp)

	my_conn.close()
	sys.exit(0)

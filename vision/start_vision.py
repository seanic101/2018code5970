#! /usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4
#
# Starts vision system
#
# This is started at system boot as a service.  If the location of the
# directory holding this file changes, it must be reflected in:
#	/lib/systemd/system/jetson.service
# If the above file is modified, run as root:
#	systemctl daemon-reload
#	systemctl start jetson
# To shut down the vision system, run as root:
#	systemctl stop jetson

import os
import sys

here = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, here + '/tcp')
sys.path.insert(0, here + '../haarcascade/powercube')
import time
import subprocess
import argparse
import logging
from multiprocessing import Process, Lock
from vtape import find_tape
from camera import Camera
import mmap

# in test we use this
#TCP_IP = '127.0.0.1'
# in competition the TX1 should be 10.59.70.12
TCP_IP = '10.59.70.12'

TCP_PORT = 5005

from server import jetson_server

def main(logf):
	# Build a mutex to protect a location instance
	mutex = Lock()
	s = (
		b"01234567890123456789012345678901234567890123456789" +
		b"01234567890123456789012345678901234567890123456789"
		)
	mm = mmap.mmap(-1, 100)
	mm.write(s)

	logging.debug("start")

	cam = Camera(0)
	cam.setup()

	# Start vision system for tape
	tape_process = Process(target = find_tape, args = (mm, mutex, cam,))
	tape_process.start()

	# Start vision system for powercube
	# XXX

	# Start server to supply roborio with values
	server_process = Process(target = jetson_server, args = (mm, mutex, TCP_IP, TCP_PORT,))
	server_process.start()

	# When server shuts down bring down others...
	server_process.join()
	tape_process.terminate()
	
	cam.shutdown()
	# close the memory map of location
	mm.close()

if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Beavertronics Jetson TX1 daemon")

	# The following is to ignore the '--pid-file' flag.  Note: 'args.pid-file' is an illegal
	# variable name and cannot be used.
	parser.add_argument('-p', '--pid-file', default='/tmp/start_vision.pid')
	parser.add_argument('-l', '--logfile', default='/tmp/start_vision.log')
	parser.add_argument('-d', '--debug', action='store_true')

	args = parser.parse_args()
	if args.debug:
		logging.basicConfig(filename='/tmp/logger.log', level=logging.DEBUG)
		logging.debug("Listening for clients on localhost...")
		TCP_IP = '127.0.0.1'
	else:
		logging.basicConfig(filename='/tmp/logger.log', level=logging.WARNING)
		

	main(args.logfile)

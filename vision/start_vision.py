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

import sys

sys.path.insert(0, 'tcp')
sys.path.insert(0, '../haarcascade/powercube')
import os
import time
import subprocess
import argparse
import logging
import daemon
from daemon import pidfile
from multiprocessing import Process, Lock
from vtape import find_tape
# XXX from location import Location
from camera import Camera
import mmap



# in competition the following is the roborio '10.59.70.177'
# in competition 

# in test we use this
#TCP_IP = '127.0.0.1'

# in competition the TX1 should be 10.59.70.12
TCP_IP = '10.59.70.12'

TCP_PORT = 5005

from server import jetson_server

debug_p = False

def main(logf):
	# Build a mutex to protect a location instance
	mutex = Lock()
	# XXX loc = Location()
#	loc = '/tmp/location'
#	with open(loc, 'wb') as f:
#		# memory map 50 chars
#		s = (
#			b"01234567890123456789012345678901234567890123456789" +
#			b"01234567890123456789012345678901234567890123456789"
#			)
#		f.write(s)
#
#	with open("/tmp/location", "r+b") as f:
#		mm = mmap.mmap(f.fileno(), 0)

	s = (
		b"01234567890123456789012345678901234567890123456789" +
		b"01234567890123456789012345678901234567890123456789"
		)
	mm = mmap.mmap(-1, 100)
	mm.write(s)

	logging.debug("start")
#	logger = logging.getLogger('jetson')
#	logger.setLevel(logging.INFO)
#
#	fh = logging.FileHandler(logf)
#	fh.setLevel(logging.DEBUG)
#
#	formatstr = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
#	formatter = logging.Formatter(formatstr)
#
#	fh.setFormatter(formatter)
#
#	logger.addHandler(fh)

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

	#logger.debug("this is a DEBUG message")
	#logger.info("this is an INFO message")
	#logger.error("this is an ERROR message")

def start_daemon(pidf, logf):
	### This launches the daemon in its context
	### XXX pidfile is a context

	# path to the directory containing start_vision
	here = os.path.dirname(os.path.realpath(__file__))
	my_uid = os.getuid()
	my_gid = os.getgid()

	with daemon.DaemonContext(
		working_directory=here,
		umask=0o002,
		stdout=open("/tmp/stdout", "wb"), stderr=open("/tmp/stderr", "wb"),
		pidfile=pidfile.TimeoutPIDLockFile(pidf),
		uid=my_uid, gid=my_gid) as context:
		main(logf)

if __name__ == "__main__":
	#reset_camera()
	parser = argparse.ArgumentParser(
		description="Beavertronics Jetson TX1 daemon")
	parser.add_argument('-p', '--pidfile', default='/tmp/start_vision.pid')
	parser.add_argument('-l', '--logfile', default='/tmp/start_vision.log')

	parser.add_argument('-d', '--debug', action='store_true')

	# to satisfy the requirement of systemd that forking process types use
	# a --daemon flag
	parser.add_argument('--daemon', action='store_true')

	args = parser.parse_args()
	if args.debug:
		logging.basicConfig(filename='logger.log', level=logging.DEBUG)
		print("Listening for clients on localhost...")
		TCP_IP = '127.0.0.1'
	else:
		logging.basicConfig(filename='logger.log', level=logging.WARNING)
		

	start_daemon(pidf=args.pidfile, logf=args.logfile)


#def reset_camera():
#	FNULL = open(os.devnull, 'w')
#	argument = '/usr/bin/nvgstcapture-1.0'
#	proc = (subprocess.Popen([argument],
#		env=dict(os.environ, DISPLAY=":0.0", XAUTHORITY="~/.Xauthority"),
#		stdout=FNULL, stderr=FNULL, stdin=subprocess.PIPE))
#
#	time.sleep(3) # <-- There's no time.wait, but time.sleep.
#
#	print "killing proc..."
#	proc.communicate('quit\n')

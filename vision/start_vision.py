#! /usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4
#
# Starts vision system

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
from location import Location

LOC = Location()

# in competition the following is the roborio
#TCP_IP = '10.59.70.2'
# in competition the TX1 should be 10.59.70.12
# in test we use this
TCP_IP = '127.0.0.1'

TCP_PORT = 5005
USING_TAPE = False
USING_CUBE = False

from server import jetson_server

debug_p = False

# Build a mutex to protect a location instance
MUTEX = Lock()

def main(logf):
	logger = logging.getLogger('jetson_tx1')
	logger.setLevel(logging.INFO)

	fh = logging.FileHandler(logf)
	fh.setLevel(logging.INFO)

	formatstr = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
	formatter = logging.Formatter(formatstr)

	fh.setFormatter(formatter)

	logger.addHandler(fh)

	# Start vision system for tape
	tape_process = Process(target = find_tape, args = (LOC,))
	tape_process.start()

	# Start vision system for powercube
	# XXX

	# Start server to supply roborio with values
	server_process = Process(target = jetson_server, args = (LOC, TCP_IP, TCP_PORT,))
	server_process.start()

	# When server shuts down bring down others...
	server_process.join()
	tape_process.terminate()

	#logger.debug("this is a DEBUG message")
	#logger.info("this is an INFO message")
	#logger.error("this is an ERROR message")

def start_daemon(pidf, logf):
	### This launches the daemon in its context
	### XXX pidfile is a context
	with daemon.DaemonContext(
		working_directory='/home/nvidia/opencv_workspace/robotgit/2018code5970/vision',
		umask=0o002,
		stdout=open("/tmp/stdout", "wb"), stderr=open("/tmp/stderr", "wb"),
		pidfile=pidfile.TimeoutPIDLockFile(pidf),
		uid=1001, gid=1001) as context:
		main(logf)
		#working_directory='/var/lib/jetson_tx1',


if __name__ == "__main__":
	#reset_camera()
	parser = argparse.ArgumentParser(
		description="Beavertronics Jetson TX1 daemon")
	parser.add_argument('-p', '--pid-file', default='/var/run/jetson_tx1/start_vision.pid')
	parser.add_argument('-l', '--log-file', default='/var/log/jetson_tx1/start_vision.log')

	args = parser.parse_args()

	start_daemon(pidf=args.pid_file, logf=args.log_file)


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

#! /usr/bin/python
# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4
#
# Starts vision system

import sys
sys.path.insert(0, 'tcp')
import os
import time
import subprocess

# Fix server.py to not start until told
#import server

def main():
	# Start vision system for tape
	# XXX

	# Start vision system for powercube
	# XXX

	# Start server to supply roborio with values
	# XXX
	print "HERE"

def reset_camera():
	FNULL = open(os.devnull, 'w')
	argument = '/usr/bin/nvgstcapture-1.0'
	proc = (subprocess.Popen([argument],
		env=dict(os.environ, DISPLAY=":0.0", XAUTHORITY="~/.Xauthority"),
		stdout=FNULL, stderr=FNULL, stdin=subprocess.PIPE))

	time.sleep(3) # <-- There's no time.wait, but time.sleep.

	print "killing proc..."
	proc.communicate('quit\n')

if __name__ == "__main__":
	reset_camera()
	print "Starting vision systems..."
	main()

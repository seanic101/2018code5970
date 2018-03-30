#! /bin/bash
# Starts up the jetson vision system
cd ~/opencv_workspace/robotgit/2018code5970/vision
rm logger.log
rm /tmp/stderr
rm /tmp/stdout
echo "Restarting server with killall command"
killall start_vision.py
killall start_vision.py
killall start_vision.py

#-d means debug mode, should not be active in competition
#/bin/systemctl restart jetson
./start_vision.py 

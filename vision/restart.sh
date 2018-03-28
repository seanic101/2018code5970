#! /bin/bash
# Starts up the jetson vision system
cd ~/opencv_workspace/robotgit/2018code5970/vision
rm logger.log
rm /tmp/stderr
rm /tmp/stdout
#/bin/systemctl restart jetson
./start_vision.py

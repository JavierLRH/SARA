#! /bin/bash

rosparam set /canusb/port /dev/ttyUSB0
rosparam set /canusb/baud 1m
rosrun canusb canusb.py

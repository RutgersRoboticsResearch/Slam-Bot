#!/bin/bash

HOST=192.168.0.101
PORT=9001

gst-launch-1.0 -vvv tcpclientsrc port=${PORT} host=${HOST} ! \
  tsdemux ! h264parse ! avdec_h264 ! xvimagesink sync=false

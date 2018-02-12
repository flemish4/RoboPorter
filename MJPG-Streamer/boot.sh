#!/bin/bash

sudo modprobe bcm2835-v4l2
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so" -o "/usr/local/lib/output_http.so -w /usr/local/www"
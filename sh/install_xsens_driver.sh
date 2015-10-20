#!/bin/bash

cd /usr/Aris/sh/xsens_driver && make
modprobe usbserial
insmod xsens_mt.ko
cd ..
echo "imu driver installed successfully"


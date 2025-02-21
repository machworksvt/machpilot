#!/bin/bash
# /usr/local/bin/init_can.sh

busybox devmem 0x0c303018 w 0xc458
busybox devmem 0x0c303010 w 0xc400

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 up type can bitrate 250000
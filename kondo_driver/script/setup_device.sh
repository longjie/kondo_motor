#!/bin/sh

sudo modprobe ftdi_sio
sudo chmod 666 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
sudo echo 165c 0006 >> /sys/bus/usb-serial/drivers/ftdi_sio/new_id
sudo echo 165c 0008 >> /sys/bus/usb-serial/drivers/ftdi_sio/new_id

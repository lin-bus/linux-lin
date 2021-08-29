#!/bin/bash

cd sllin
make -j4

sudo insmod ./sllin.ko
sudo ldattach 28 /dev/ttyS0
sudo ip link set sllin0 up
echo $(ip link show dev sllin0)

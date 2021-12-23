#!/bin/bash

sudo cp DW0038.rules /etc/udev/rules.d
sudo cp DW009A.rules /etc/udev/rules.d
sudo cp DW02D6.rules /etc/udev/rules.d
sudo cp DW1632.rules /etc/udev/rules.d
sudo cp DW2D9C.rules /etc/udev/rules.d
sudo cp DW43EB.rules /etc/udev/rules.d
sudo cp DW47FC.rules /etc/udev/rules.d
sudo cp DW4806.rules /etc/udev/rules.d
sudo cp DW4814.rules /etc/udev/rules.d
sudo cp DW482E.rules /etc/udev/rules.d
sudo cp DW4848.rules /etc/udev/rules.d
sudo cp DW4984.rules /etc/udev/rules.d
sudo cp dwm1001.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
sudo usermod -a -G dialout $USER
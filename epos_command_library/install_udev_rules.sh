#!/bin/bash

echo ""
echo "This script copies a udev rule for epos devices"
echo ""

sudo cp `rospack find eposx_library`/{99-ftdi.rules,99-epos4.rules} /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
#!/bin/bash

# Back-up the location of the starting directory
STARTING_DIR=$PWD

# Turn on the two needed I2C buses
sudo echo 'dtparam=i2c_arm=on' >> /boot/config.txt
sudo echo 'dtoverlay=i2c-gpio,bus=3' >> /boot/config.txt

# Install python dependencies
sudo apt install python3-pyvisa python3-pyvisa-py python3-numpy python3-sklearn
sudo apt install python3-pyqt5 python3-yaml python3-serial python3-pyqtgraph

# Install the NAU7802 library
cd ~
git clone https://github.com/ljaworski88/nau7802-py.git
cd nau7802-py
pip3 install .

# Install the SF04 sensor library ot control the Sensirion SLG-150 flow sensor
cd ~
git clone https://github.com/ljaworski88/sensirion-sf04-python.git
cd sensirion-sf04-python
sudo rm /boot/overlays/i2c-gpio.dtbo
sudo cp i2c-gpio.dtbo /boot/overlays/i2c-gpio.dtbo
pip3 install .

# Place a symbolic link to the flow reader on the desktop
cd ~/Desktop
ln -s home/pi/flow_reader/src/python/flow_reader.py

# Return to the install directory
cd $STARTING_DIR

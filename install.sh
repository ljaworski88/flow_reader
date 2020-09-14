#!usr/bin/env/ bash

apt install python3-pyvisa python3-pyvisa-py python3-sklearn python3-yaml
apt install python3-pyqt5 python3-numpy python3-serial python3-pyqtgraph
cd ~
git clone https://github.com/ljaworski88/nau7802-py.git
rm /boot/overlays/i2c-gpio.dtbo
cd nau7802-py
cp i2c-gpio.dtbo /boot/overlays/i2c-gpio.dtbo
pip3 install .
echo 'dtparam=i2c_arm=on' >> /boot/config.txt
echo 'dtoverlay=i2c-gpio,bus=3' >> /boot/config.txt
cd ~
git clone https://github.com/ljaworski88/sensirion-sf04-python.git
cd sensirion-sf04-python
pip3 install .


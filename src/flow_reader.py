#!/usr/bin/env python3

from sf04 import *
from time import sleep
from os import system


with SMBus(3) as bus:
    reset_sensor(bus)
    print("Sensor reset!")
    sleep(0.5)

    sensor, serial_number, _, _ = read_product_info(bus)
    scale_factor, units, _, _ = read_scale_and_unit(bus)
    set_read_data()
    while True:
        raw_flow_reading, _, crc_result = read_raw_data(bus, True)
        scaled_flow_reading = scale_reading(raw_flow_reading, scale_factor)
        if crc_result:
            print('The current flow rate is: {} {}'.format(scaled_flow_reading, units))
        else:
            print('Error! Error!')
        sleep(1)
        system('clear')

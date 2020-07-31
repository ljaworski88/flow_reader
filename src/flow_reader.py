#!/usr/bin/env python3

from sf04 import *
from time import sleep, time
from collections import deque
from statistics import mean
from os import system


with SMBus(3) as bus:
    reset_sensor(bus)
    print("Sensor reset!")
    sleep(0.5)
    system('clear')

    sensor, serial_number, _, _ = read_product_info(bus)
    scale_factor, units, _, _ = read_scale_and_unit(bus)
    print('Sensor is:')
    print(sensor)
    print('Serial Number:')
    print(serial_number)
    print('Scale Factor:')
    print(scale_factor)
    print('Units:')
    print(units)
    sleep(3)

    set_read_data(bus)
    timeout_buffer = deque([], 3000)
    while True:
        raw_flow_reading, _, _ = read_raw_data(bus, True)
        scaled_flow_reading = scale_reading(raw_flow_reading, scale_factor)
        print('The current flow rate is: {} {}'.format(scaled_flow_reading, units))
        if 3100 < scaled_flow_reading < 3500:
            timeout_buffer.append((time(), scaled_flow_reading))
            if len(timeout_buffer) > 2500:
                beggining_average = mean(list(timeout_buffer)[:100])
                ending_average = mean(list(timeout_buffer)[-100:])
                if abs(beggining_average-ending_average) < 100:
                    print('ALL DONE!!')
                else:
                    print('Approaching the end')
        sleep(0.25)
        system('clear')

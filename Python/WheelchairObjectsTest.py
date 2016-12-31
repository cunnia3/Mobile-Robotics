# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 14:28:48 2016

@author: cunnia3
"""
from WheelchairObjects import *
import matplotlib.pyplot as plt
import numpy as np

## MOTOR TEST
start_state = np.zeros([2,1])
start_state[1] = 100
my_motor = Motor(start_state)

time = 0
sense_dt = .1 # time between measurements
for i in range(10):
    time+=sense_dt
    my_motor.measurement_update(my_motor.get_noisy_measurement(), time)
    
plt.plot(my_motor.state_history)
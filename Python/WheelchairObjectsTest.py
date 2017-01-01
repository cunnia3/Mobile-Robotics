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
start_state[1] = -10
my_motor = SimulatedMotor(start_state)

time = 0
sense_dt = .1 # time between measurements
for i in range(1000):
    time+=sense_dt
    my_motor.update(time)
    
plt.plot(my_motor.state_history)
plt.plot(my_motor.control_history, 'r')
import numpy as np
import math
import copy
import scipy.integrate as integrate
from scipy import linalg

# Adapted from http://greg.czerniak.info/guides/kalman1/
class KalmanFilterLinear:
    def __init__(self,_A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.current_state_estimate = _x # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.
        
    def get_current_state(self):
        return self.current_state_estimate
        
    def sense_update(self, measurement_vector):
        """ Update Kalman filter if sensing information is received """
       #--------------------------Observation step-----------------------------
        innovation = measurement_vector - np.dot(self.H, self.current_state_estimate)
        innovation_covariance = np.dot(np.dot(self.H, self.current_prob_estimate), np.transpose(self.H)) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = np.dot(np.dot(self.current_prob_estimate, np.transpose(self.H)), np.linalg.inv(innovation_covariance))
        self.current_state_estimate = self.current_state_estimate + np.dot(kalman_gain, innovation)
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = np.dot((np.eye(size)-np.dot(kalman_gain,self.H)), self.current_prob_estimate)
        
    def step(self, control_vector):
        """ Step without sensing """
        self.current_state_estimate = np.dot(self.A, self.current_state_estimate) + np.dot(self.B, control_vector)
        self.current_prob_estimate = np.dot(np.dot(self.A, self.current_prob_estimate), np.transpose(self.A)) + self.Q

class Motor:
    """ Base class for a motor object, is implemented to be a simulated motor
        Extend for real motor"""
    def __init__(self, starting_state, track_history = True):
        self.track_history = track_history
        self.dt = .005
        self.k1 = -2           # motor constant
        self.k2 = 2            # motor constant
        self.state = starting_state
        self.saturation = 20     # absolute saturation velocity
        self.Q = np.eye(2)*.001  # variance of process noise
        self.R = np.eye(2)      # variance of sensing noise
        
        self.t_last_m = 0
        self.u = 0
        
        self.desired_speed = 0
        self.state_history = []
        self.control_history = []
        
        ## State space
        continuous_A = self.k1
        continuous_B = self.k2
        
        # Discrete state space
        self.A = math.exp(continuous_A * self.dt)
        self.B = (1.0/self.k1)*(self.A - 1)*self.k2
        self.C = np.eye(2)
        
        self.kf = KalmanFilterLinear(self.A, self.B, self.C, self.state, np.zeros([2,2]), self.Q, self.R)
        self.K = np.ones([1,2])
        self.K[0,0] = 0
        self.K[0,1] = 3
        
        self.set_desired_speed(10)
        
    def measurement_update(self, measurement, time_now):
        """ Incorporate a measurement, find new control """
        # update motor speed with time difference, DISCRETIZED!
        time_since_last_measurement = -(self.t_last_m - time_now)
        num_steps = int(time_since_last_measurement / self.dt)
        self.t_last_m = time_now
        
        for i in range(num_steps):
            self._step_voltage(self.u)
        
        self.kf.sense_update(measurement)
        self._update_control()
        
    def get_noisy_measurement(self):
        """ WILL NOT BE PRESENT IN NON-SIMULATED"""
        mean = [0, 0]
        return self.state + np.random.multivariate_normal(mean, self.R).reshape([2,1])
        
    def set_desired_speed(self, desired_speed):
        """ Set desired speed for the motor, calculate steady state current 
            with feed forward"""
        self.set_point = desired_speed
        self.feed_forward_u = -self.k1*desired_speed/self.k2
        
    def _step_voltage(self, voltage):
        """ apply a voltage to the motor for specified time, simulated has to update true state """
        # UPDATE REAL STATE
        mean = [0, 0]
        self.state = np.dot(self.A, self.state) + np.dot(self.B, self.u) + np.random.multivariate_normal(mean, self.Q).reshape([2,1])
        if self.track_history:
            self.state_history.append(self.state[1])
            self.control_history.append(voltage)

        # UPDATE KALMAN FILTER
        self.kf.step(voltage)
            
    def _update_control(self):
        self.u = np.dot(self.K, self.set_point-self.state) + self.feed_forward_u
        if self.u > self.saturation:
            self.u = self.saturation
            
        if self.u < -self.saturation:
            self.u = -self.saturation
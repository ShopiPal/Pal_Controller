
import numpy as np
# GLOBAL PARAMETERS
MAX_PWM = 255
MIN_PWM = 0

MAX_V = 12
MIN_V = 0
# PID controller class,
class PID(object):
    def __init__(self,KP ,KI, KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.kb = 0.5
        self.error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
        self.output = 0
        self.ERROR_MIN = 0.02

        self.prop_term = 0
        self.integral_term = 0 

        self.derivative_term_filtered = 0
        self.tau = 0.035
    
    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def windup_handle( self):
            v_output_before_sat = self.prop_term + self.ki*self.integral_error + self.derivative_term_filtered
            if v_output_before_sat > MAX_V:
                return (self.ki - self.kb*(MAX_V - v_output_before_sat))*self.integral_error
            elif v_output_before_sat < MIN_V:
                return (self.ki + self.kb*(MIN_V - v_output_before_sat))*self.integral_error
            else:
                return self.ki*self.integral_error

    def saturate(self,v_output):
        if v_output >= MAX_V:
            v_output = MAX_V
        elif v_output <= MIN_V:
            v_output = MIN_V
        return v_output
        

    def compute(self,current_speed,target_speed,time_interval):
        self.error = target_speed - current_speed
        
        
        self.derivative_error = (self.error-self.last_error)/time_interval
        self.derivative_term = self.kd*self.derivative_error
        self.derivative_term_filtered = (
             (self.tau/(self.tau + time_interval))*self.derivative_term_filtered +
             (time_interval/(self.tau+time_interval))*self.derivative_term
         )
        self.last_error = self.error
        
        self.integral_error += self.error * time_interval
        self.integral_term = self.ki*self.integral_error
        self.integral_term = self.windup_handle()
        
        
        if np.abs(self.error) < self.ERROR_MIN:
           self.error = 0
        self.prop_term = self.kp*self.error
        
        v_output = self.prop_term + self.integral_term + self.derivative_term
        v_output = self.prop_term + self.integral_term + self.derivative_term_filtered
        v_output = self.saturate(v_output)

        self.output = int(self.map_range(v_output,MIN_V ,MAX_V,MIN_PWM,MAX_PWM))
        return self.output


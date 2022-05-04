from ossaudiodev import control_labels
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL PARAMETERS
MAX_PWM = 255
MIN_PWM = 0

MAX_VOLT = 12
MIN_VOLT = 0

# PID controller class,
class PID(object):
    def __init__(self,KP ,KI ,KD ):
        # constants
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.tau = 0.6
        
        # inegrator windup params
        self.ki_windup = 0
        self.kb = 2  # <<< need to tune
        self.windup_method = None 
        
        
        # error varoables
        self.prev_errors = [ 0, 0]
        self.error = 0
        self.integral_error = 0
        self.last_integral_error = 0


        self.prop_term = 0
        self.integral_term = 0
        self.derivative_term = 0
        self.derivative_term_filtered = 0
        self.v_output = 0
        self.output = 0
        self.map_range = interp1d([MIN_VOLT,MAX_VOLT],[MIN_PWM,MAX_PWM])
    
    
    def compute( self, current_speed, target_speed, time_interval):
        # calculating proportional term
        self.error = target_speed - current_speed
        self.prop_term = self.kp * (self.error - self.prev_errors[0])
        # calculate derviative term
        self.derivative_term = (self.kd*(self.error-2*self.prev_errors[0]+self.prev_errors[1]))/time_interval
        # filtering dervative term
        self.derivative_term_filtered = (
            (self.tau/(self.tau+ time_interval))*self.derivative_term_filtered +
            (time_interval/(self.tau+time_interval))*self.derivative_term
        )
        
        
        # calculating integral term with anti-windup methods 
        self.last_integral_error = self.integral_error
        self.integral_error += self.error * time_interval 
        print("integrator error before windup = " , self.integral_error)
        self.integral_term = self.ki*self.integral_error
        # Calculating PID controller output u[n] before anti-windup
        v_output = self.prop_term + self.integral_term + self.derivative_term_filtered
        print("v_output before windup = ", v_output)
        
        if  (v_output > MAX_VOLT or v_output < MIN_VOLT) and self.windup_method != None:
            print("winduping")
            self.windup_integral_term = self.windup_handle(v_output)
            print("ki = ",self.ki_windup)
            # Calculating PID controller output u[n] after anti-windup
            v_output = self.prop_term + self.windup_integral_term + self.kd*self.derivative_term_filtered
            print("v_output after winduping = ",v_output)
        
        self.v_output = v_output
        self.saturate()
        self.output = int(self.map_range(self.v_output))
        
        # Updating previous time step errors e[n-1], e[n-2]
        self.prev_errors[1] = self.prev_errors[0]
        self.prev_errors[0] = self.error

        return self.output

    def windup_handle( self,v_output_before_sat):
        
        if self.windup_method == "clamp" and np.sign(self.integral_error)==np.sign(v_output_before_sat):
            print("clamping windup")
            return self.ki*self.last_integral_error
        elif self.windup_method == "BC":
            print("BC")
            if v_output_before_sat > MAX_VOLT:
                return (self.ki - self.kb*(MAX_VOLT - v_output_before_sat))*self.integral_error
            else:
                return (self.ki + self.kb*(MIN_VOLT - v_output_before_sat))*self.integral_error
        else:
            print("in clamping method but different signs")
            return self.ki*self.integral_error

    def saturate(self):
        if self.v_output >= MAX_VOLT:
            print("saturated")
            self.v_output = MAX_VOLT
        elif self.v_output <= MIN_VOLT:
            print("saturated")
            self.v_output = MIN_VOLT
        else:
            pass

control = PID(1,1,10)

time = np.linspace(0,2,11)
velocity_input = np.linspace(0,1,11)
target_velocity = 1
dt = 0.2
pwm = np.zeros(11)
for i in range(11):
    pwm[i] = control.compute(velocity_input[i],target_velocity,dt)
    print("pwm  = ", pwm[i])

plt.plot(time,pwm)
plt.show()
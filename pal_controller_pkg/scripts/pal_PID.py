# GLOBAL PARAMETERS
MAX_PWM = 255
MIN_PWM = 0

# PID parameters
KP = 0.0155
KI = 0.026
KD = 0.00146
# PID controller class,
class PID(object):
    def __init__(self,KP =0.0155 ,KI=0.026,KD=0.00146):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
        self.output = 0
    
    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

    def compute(self,current_speed,target_speed,time_interval=0.2):
        self.error = target_speed - current_speed
        self.integral_error += self.error * time_interval
        self.derivative_error = (self.error-self.last_error)/time_interval
        self.last_error = self.error
        v_output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
        self.output = self.map_range(v_output,5,11,0,255)
        if self.output >= MAX_PWM:
            self.output = MAX_PWM
        elif self.output <= MIN_PWM:
            self.output = MIN_PWM
        return self.output
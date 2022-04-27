# GLOBAL PARAMETERS
MAX_PWM = 255
MIN_PWM = 0

# PID parameters
KP = 0.0155
KI = 0.026
KD = 0.00146
# PID controller class,
class PID(object):
    def __init__(self,KP,KI,KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.last_error = 0
        self.output = 0

    def compute(self,current_speed,target_speed,time_interval=0.2):
        self.error = target_speed - current_speed
        self.integral_error += self.error * time_interval
        self.derivative_error = (self.error-self.last_error)/time_interval
        self.last_error = self.error
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
        if self.output >= MAX_PWM:
            self.output = MAX_PWM
        elif self.output <= MIN_PWM:
            self.output = MIN_PWM
        return self.output
#!/usr/bin/env python
import rospy


#from std_msgs.msg import Int64
#from std_msgs.msg import Float64
from std_msgs.msg import Int16

class Controller:

    def __init__(self):
        self.leftPwmPublisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=10)
<<<<<<< HEAD
        #self.rightPwmPublisher = rospy.Publisher("/right_pwm",Int16,queue_size=10)
        self.leftEncoderSub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        #self.rightEncoderSub = rospy.Subscriber("/encoder_right_ticks",Int16,rightEncoder_callback)

        ## set parameters
        
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 200

        self.left_encoder_value = 0

=======
        self.rightPwmPublisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=10)
        self.leftEncoderSub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        self.rightEncoderSub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)

        ## set parameters
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 60
        self.left_encoder_value = 0
        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 60
        self.right_encoder_value = 0
>>>>>>> origin/controller
    
    def publish_pwm(self):
          
        #self.pwm_left_out.data =  palPID(.....)
        #self.pwm_right_out.data = palPID(.....) 
        
<<<<<<< HEAD

        self.leftPwmPublisher.publish(self.pwm_left_out)
        #self.rightPwmPublisher.publish(pwm_right_out)
=======
        self.leftPwmPublisher.publish(self.pwm_left_out)
        self.rightPwmPublisher.publish(self.pwm_right_out)
>>>>>>> origin/controller
        
    def leftEncoder_callback(self,msg):
        self.left_encoder_value = msg.data
    
    def rightEncoder_callback(self,msg):
        self.right_encoder_value = msg.data

    def stop_robot():
        return            

if __name__ == '__main__':
    rospy.init_node('pal_controller_node', anonymous=True)
    
    pal_control = Controller()
       
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pal_control.publish_pwm()
        
        rate.sleep()

    
    
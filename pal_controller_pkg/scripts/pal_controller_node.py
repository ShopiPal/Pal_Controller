#!/usr/bin/env python
import rospy
from math import pi , cos , sin 
#from std_msgs.msg import Int64
#from std_msgs.msg import Float64
from std_msgs.msg import Int16
from geometry_msgs import Twist , Pose , Point , Quaternion , vector3
from nav_msgs import Odometry 
import tf
from tf.broadcaster import TransformBroadcaster


class Controller:

    def __init__(self):
        self.left_pwm_publisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=10)
        self.right_pwm_publisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=10)
        self.left_encoder_sub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        self.right_encoder_sub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist , queue_size = 10)
        self.odom_publisher = rospy.Publisher("/odom" , Odometry , queue_size= 10)
        

        ## init msgs
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 60
        self.last_left_encoder_value = 0 
        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 60
        self.last_right_encoder_value = 0

        # init cmd msg
        self.cmd = Twist()

        # init parameters
        #self.R = 
        #self.L =
        #self.N =
        #self.x = 
        #self.y =
        #self.theta =





    def publish_pwm(self):
          
        #self.pwm_left_out.data =  palPID(.....)
        #self.pwm_right_out.data = palPID(.....) 
        
        self.left_pwm_publisher.publish(self.pwm_left_out)
        self.right_pwm_publisher.publish(self.pwm_right_out)
        
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

    
    
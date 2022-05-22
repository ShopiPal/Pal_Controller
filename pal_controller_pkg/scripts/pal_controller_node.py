#!/usr/bin/env python

# import libraries & ros plugins & msgs
import numpy as np
import rospy
from math import pi , cos , sin , asin
from std_msgs.msg import Float64 ,Float32 , Int16
from pal_controller_pkg.srv import PwmVal , PwmValResponse
from std_srvs.srv import SetBool , SetBoolResponse
from geometry_msgs.msg import Pose ,Twist , Point , Quaternion , Vector3
from nav_msgs.msg import Odometry 
import tf
from tf.broadcaster import TransformBroadcaster
from pal_PID import PID
import time


''' 
creation class of type 'controller'
will help to deal with sensors inputs/outputs and kinematics computations
publishers:  left/right_pwm 
            odom
subscribers: left/right_encoder 
             cmd_vel
services:   set_pwm
            motors_stop
methods: 
            shutdownhook - shutdown process will call motors_stop service
            control_vel_callback - handle with cmd_vel topic
            publish - publish topics
            set_pwm_callback - set pwm values to the topics
            stop_callback - handle with stop service
            stop - motors stop logic
            left/rightEncoder_callback - hande with encoders topic 
            update_pose - kinematics computations
            calc_ticks - compute ticks delta
            odom_msg_init - init odom msg format
'''
class Controller:

    def __init__(self):
        
        ## init publisher and subscribers
        self.left_pwm_publisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=100) 
        self.right_pwm_publisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=100) 
        # self.left_encoder_sub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        # self.right_encoder_sub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist,self.control_vel_callback)
        self.odom_publisher = rospy.Publisher("/odom" , Odometry , queue_size = 1000)

       
        self.vr_current_filter_sub = rospy.Subscriber("/velocity/vr_current/filter",Float32,self.vr_current_filter_callback)
        self.vr_current_raw_sub = rospy.Subscriber("/velocity/vr_current/raw",Float32,self.vr_current_raw_callback)
        self.vl_current_filter_sub = rospy.Subscriber("/velocity/vl_current/filter",Float32,self.vl_current_filter_callback)
        self.vl_current_raw_sub = rospy.Subscriber("/velocity/vl_current/raw",Float32,self.vl_current_raw_callback)

        self.delta_distance_left_sub = rospy.Subscriber("/delta_distance/dl",Float32,self.delta_distance_left_callback)
        self.delta_distance_right_sub = rospy.Subscriber("/delta_distance/dr",Float32,self.delta_distance_right_callback)

        self.vr_target_publisher = rospy.Publisher("/velocity/vr_target",Float32,queue_size=100)
        self.vl_target_publisher = rospy.Publisher("/velocity/vl_target",Float32,queue_size=100)

        self.t_0 = time.time()

        ## init services
        self.set_pwm_service = rospy.Service('motors/set_pwm', PwmVal , self.set_pwm_callback)
        self.motors_stop_service = rospy.Service('motors/stop', SetBool , self.stop_callback)

        ## init time variables 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        ## init msgs ##
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 0        


        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 0      


        # init kinematics parameters
        self.R = 0.125/2    
        self.L = 0.472       # need to update
        self.x = 0      
        self.y = 0 
        self.theta = 0.00000000001

        ## init cmd_vel
        self.vr_current_filter = 0
        self.vr_current_raw = 0
        self.vl_current_filter = 0
        self.vl_current_raw = 0
        
        self.cmd_vel = Twist()
        self.vr_target=0
        self.vl_target=0
        
        ## init distance robot passed
        self.delta_distance_left = 0
        self.delta_distance_right = 0

        ## init odom and tf
        self.odom = Odometry()
        self.odom_broadcaster = TransformBroadcaster()

        ## shutdownhook process
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        ## PID init
        self.pal_control_l = PID(16 ,52 ,0.75) ## need to tune
        self.pal_control_r = PID(16 ,52 ,0.75)
        
       # self.update_pose()

    def shutdownhook(self):
        rospy.loginfo("shutting down")
        self.ctrl_c = True
        self.stop()
        

    def control_vel_callback(self,msg):
        ## calculate cmd_motors_vels from a given cmd_vel and insert to the pid controller for pwm output
        self.cmd_vel = msg
        self.vr_target = (2*self.cmd_vel.linear.x + self.cmd_vel.angular.z * self.L)/2
        self.vl_target = (2*self.cmd_vel.linear.x - self.cmd_vel.angular.z * self.L)/2
        
            
    def publish(self):
        
        # publish pwm
        self.left_pwm_publisher.publish(self.pwm_left_out)
        self.right_pwm_publisher.publish(self.pwm_right_out)
        # publish velocities for test only
        self.vr_target_publisher.publish(self.vr_target)
        self.vl_target_publisher.publish(self.vl_target)

        # publish odom
        self.odom_publisher.publish(self.odom)

    def set_pwm_callback(self, request):
        response = PwmValResponse()
        if ((request.pwm_left<-255)or(request.pwm_left>255)or(request.pwm_right<-255)or(request.pwm_right>255)):
            response.success = False
        else:
            self.pwm_left_out.data = request.pwm_left
            self.pwm_right_out.data = request.pwm_right
            response.success = True
        return response

    def stop_callback(self, request):
        
        response = SetBoolResponse()
        if request.data:
            self.stop()
            rospy.sleep(0.5)
            if ((self.last_left_encoder_value == self.current_left_encoder_value) and (self.last_right_encoder_value == self.current_right_encoder_value)):
                response.success = True
                response.message = "motors stopped"

            else:
                response.success = False
                response.message = "motors didn't stop from some reason"

        else:
            response.success = False
            response.message = "enter 'true' to stop the motors" 

        return response

    def stop(self):
        time_relation = 1 # [sec]
        pwm_left = self.pwm_left_out.data
        pwm_right = self.pwm_right_out.data
        last_time = rospy.Time.now()
        current_time = rospy.Time.now()
        dt  = (current_time - last_time).to_sec()
        rate = rospy.Rate(10)
        while(dt < time_relation):  ### need to publish pwm? 
            dt  = (current_time - last_time).to_sec()
            self.pwm_left_out.data = ((-pwm_left*dt)/time_relation) + pwm_left
            self.pwm_right_out.data = ((-pwm_right*dt)/time_relation) + pwm_right    
            current_time = rospy.Time.now()
            rate.sleep()

        self.pwm_left_out.data = 0
        self.pwm_right_out.data = 0

    
    def vr_current_filter_callback(self,msg):
        self.vr_current_filter = msg.data
    
    def vr_current_raw_callback(self,msg):
        self.vr_current_raw = msg.data

    def vl_current_filter_callback(self,msg):
        self.vl_current_filter = msg.data
    
    def vl_current_raw_callback(self,msg):
        self.vl_current_raw = msg.data

    def delta_distance_right_callback(self,msg):
        self.delta_distance_right = msg.data

    # def leftEncoder_callback(self,msg):
    #     self.current_left_encoder_value = msg.data
    
    # def rightEncoder_callback(self,msg):
    #     self.current_right_encoder_value = msg.data

    def update_pose(self):

            self.current_time = rospy.Time.now()
            
            v =  (self.vl_current_filter + self.vr_current_filter )/2 # linear

            rospy.loginfo("linear velocity [m/s] = %s" , v)
            
            w = asin((self.vr_current_filter  - self.vl_current_filter)/self.L) # angular
            rospy.loginfo("angular velocity [rad/s] = %s" , w)

            if (w > pi):
                w = w - 2*pi
            if ( w < -pi ):
                w = w + 2*pi
            
            ## update movments
            delta_distance = (self.delta_distance_right + self.delta_distance_left)/2
            delta_theta = (self.delta_distance_right - self.delta_distance_left)/self.L
            ## update movments relate to axises
            delta_x = delta_distance * cos(self.theta)
            delta_y = delta_distance * sin(self.theta)
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            ## handle with theta limits
            if (self.theta > pi):
                self.theta = self.theta - 2*pi
            if (self.theta < -pi ):
                self.theta = self.theta + 2*pi
            rospy.loginfo("theta [rad] = %s" , self.theta)

            odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta)

            # publish transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x , self.y , 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

            # publish odom over ros
            self.odom_msg_init(v,w,odom_quat)

            ## set output pwm from the pid 
            direction_r = np.sign(self.vr_target) #1.0 for Forward, -1.0 for Revese
            direction_l = np.sign(self.vl_target) #1.0 for Forward, -1.0 for Revese
            direction_r_current = np.sign(self.vr_current_filter) #1.0 for Forward, -1.0 for Revese
            direction_l_current = np.sign(self.vl_current_filter) #1.0 for Forward, -1.0 for Revese



            self.pwm_right_out.data = direction_r*self.pal_control_r.compute(direction_r_current*self.vr_current_filter,direction_r*self.vr_target, 0.01) #fill dt as the arduino sample time
            self.pwm_left_out.data = direction_l*self.pal_control_l.compute(direction_l_current*self.vl_current_filter,direction_l*self.vl_target, 0.01) #fill dt as the arduino sample time
            
            #self.pwm_right_out.data = 70 + 60*sin(1*pi*(time.time()-self.t_0)) 
            #self.pwm_left_out.data = 70 + 60*sin(1*pi*(time.time()-self.t_0)) 

            
            
            print("\n")

    def odom_msg_init(self,v,w,odom_quat):
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        # set the position and orientation
        self.odom.pose.pose = Pose(Point(self.x,self.y,0) , Quaternion(*odom_quat))
        # set velocity
        self.odom.twist.twist = Twist(Vector3(v,0,0), Vector3(0,0,w))
        

'''
Main process implementation
Init:   ros node
        Controller object
        rate val = 30 hz
Loop:   update pose
        publish topics
'''
if __name__ == '__main__':
    rospy.init_node('pal_controller_node', anonymous=True)    
    pal_control = Controller()
    rate = rospy.Rate(30)
    while not pal_control.ctrl_c:
       pal_control.update_pose()
       pal_control.publish()
       rate.sleep()

    #rospy.spin()
    
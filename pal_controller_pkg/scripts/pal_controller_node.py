#!/usr/bin/env python
import rospy
from math import pi , cos , sin 
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from pal_controller_pkg.srv import PwmVal , PwmValResponse
from std_srvs.srv import SetBool , SetBoolResponse
from geometry_msgs.msg import Pose ,Twist , Point , Quaternion , Vector3
from nav_msgs.msg import Odometry 
import tf
from tf.broadcaster import TransformBroadcaster



class Controller:

    def __init__(self):
        
        ## init publisher and subscribers
        self.left_pwm_publisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=1000) 
        self.right_pwm_publisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=1000) 
        self.left_encoder_sub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        self.right_encoder_sub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist,self.control_vel_callback)
        self.odom_publisher = rospy.Publisher("/odom" , Odometry , queue_size = 1000)

        ## init services
        self.set_pwm_service = rospy.Service('motors/set_pwm', PwmVal , self.set_pwm_callback)
        self.motors_stop_service = rospy.Service('motors/stop', SetBool , self.stop_callback)


        # init encoder max and min
        self.encoder_minimum = -32768
        self.encoder_maximum = 32767

        ## init time 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        
        ## init msgs
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 0     ## for now insert here to set velocity    
        self.current_left_encoder_value = 0
        self.last_left_encoder_value = 0
        
        self.lwheel_rpm = Float64()
        self.lwheel_rpm.data = 0

        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 0      ## for now insert here to set velocity
        self.last_right_encoder_value = 0
        self.current_right_encoder_value = 0
        self.rwheel_rpm = Float64()
        self.rwheel_rpm.data = 0


        # init parameters
        self.R = 0.127/2    
        self.L = 0.476       # need to update
        self.N = 480       
        self.x = 0      
        self.y = 0 
        self.theta = 0

        ## init cmd_vel
        self.cmd_vel = Twist()

        ## init odom and tf
        self.odom = Odometry()
        self.odom_broadcaster = TransformBroadcaster()

        ## shutdownhook process
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

       # self.update_pose()

    def shutdownhook(self):
        rospy.loginfo("shutting down")
        self.stop()
        self.ctrl_c = True

    def control_vel_callback(self,msg):
        ## calculate cmd_motors_vels from a given cmd_vel and insert to the pid controller for pwm output
        self.cmd_vel = msg

        #self.pwm_left_out.data =  palPID(.....)
        #self.pwm_right_out.data = palPID(.....)
        pass

    def publish(self):
        
        # publish pwm
        self.left_pwm_publisher.publish(self.pwm_left_out)
        self.right_pwm_publisher.publish(self.pwm_right_out)

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
        rate = rospy.Rate(5)
        while(dt < time_relation):
            dt  = (current_time - last_time).to_sec()
            self.pwm_left_out.data = ((-pwm_left*dt)/time_relation) + pwm_left
            self.pwm_right_out.data = ((-pwm_right*dt)/time_relation) + pwm_right    
            current_time = rospy.Time.now()
            rate.sleep()

        self.pwm_left_out.data = 0
        self.pwm_right_out.data = 0



    def leftEncoder_callback(self,msg):
        self.current_left_encoder_value = msg.data
    
    def rightEncoder_callback(self,msg):
        self.current_right_encoder_value = msg.data

    def calc_ticks(self,current_ticks , last_ticks):

        if((current_ticks>0)and(last_ticks<0)) and (current_ticks - last_ticks)>32767: #passing Reverse from min to max
            return ((current_ticks - self.encoder_maximum)+(self.encoder_minimum-last_ticks))
        elif((current_ticks<0)and(last_ticks>0))and (current_ticks - last_ticks)<-32767: #passing Forward from max to min
            return ((current_ticks - self.encoder_minimum)+(self.encoder_maximum-last_ticks))
        else:
            return current_ticks - last_ticks


    def update_pose(self):

            current_lticks = self.current_left_encoder_value
            last_lticks = self.last_left_encoder_value
            delta_lticks = self.calc_ticks(current_lticks,last_lticks)
            rospy.loginfo("left current ticks is = %s , last ticks is = %s" , current_lticks ,last_lticks )
            rospy.loginfo("left delta ticks [ticks] = %s" , delta_lticks)

            current_rticks = self.current_right_encoder_value
            last_rticks = self.last_right_encoder_value
            delta_rticks =  self.calc_ticks(current_rticks,last_rticks)
            rospy.loginfo("right delta ticks [ticks] = %s" , delta_rticks)

            dl =  2*pi*self.R*delta_lticks/self.N 
            rospy.loginfo("left wheel distance [m] = %s" ,dl)
            dr = 2*pi*self.R*delta_rticks/self.N
            rospy.loginfo("right wheel distance [m] = %s" ,dr)

            dc = (dl + dr)/2
            rospy.loginfo("total wheel distance [m] = %s" ,dc)

            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            rospy.loginfo("dt [s] = %s" , dt)

            vl = dl/dt
            vr = dr/dt       
            v =  (vl + vr)/2
            rospy.loginfo("linear velocity [m/s] = %s" , v)
           
            w = (vr - vl)/self.L
            rospy.loginfo("angular velocity [deg/s] = %s" , w)

            delta_x = dc * cos(self.theta)
            delta_y = dc * sin(self.theta)
            delta_theta = w

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            if (self.theta > (180)):
                self.theta = self.theta - 360
            if (self.theta <= -180 ):
                self.theta = self.theta + 360

            rospy.loginfo("theta [deg] = %s" , self.theta)

            odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta)
            #quaternion = Quaternion()
		    #quaternion.x = 0 
		    #quaternion.y = 0
		    #quaternion.z = sin(self.theta / 2.0)
		    #quaternion.w = cos(self.theta / 2.0)

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

            # reset time and encoders current+last variables
            self.last_time = self.current_time
            self.last_left_encoder_value  = self.current_left_encoder_value
            self.last_right_encoder_value = self.current_right_encoder_value

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
        


         

if __name__ == '__main__':
    rospy.init_node('pal_controller_node', anonymous=True)
    
    pal_control = Controller()
   
    rate = rospy.Rate(5)
    
    while not pal_control.ctrl_c:
       
       pal_control.update_pose()
       
       pal_control.publish()
       
       rate.sleep()

    #rospy.spin()
    
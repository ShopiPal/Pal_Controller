#!/usr/bin/env python
import rospy
from math import pi , cos , sin 
#from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose,Twist   , Point , Quaternion , Vector3
from nav_msgs.msg import Odometry 
import tf
from tf.broadcaster import TransformBroadcaster



class Controller:

    def __init__(self):
        
        ## init publisher and subscribers
        self.left_pwm_publisher = rospy.Publisher("/left_motor_pwm",Int16,queue_size=10) 
        self.right_pwm_publisher = rospy.Publisher("/right_motor_pwm",Int16,queue_size=10) 
        self.left_encoder_sub = rospy.Subscriber("/encoder_left_ticks",Int16,self.leftEncoder_callback)
        self.right_encoder_sub = rospy.Subscriber("/encoder_right_ticks",Int16,self.rightEncoder_callback)
        self.left_wheel_rpm_pub = rospy.Publisher("/left_wheel_rpm", Float64 , queue_size = 10)
        self.right_wheel_rpm_pub = rospy.Publisher("/right_wheel_rpm", Float64 , queue_size = 10)
        self.cmd_vel = rospy.Subscriber("/cmd_vel",Twist,self.control_vel_callback)
        self.odom_publisher = rospy.Publisher("/odom" , Odometry , queue_size = 10)
        
        self.rate = rospy.Rate(10)

        # init encoder max and min
        self.encoder_minimum = -32768
        self.encoder_maximum = 32767

        ## init time 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        
        ## init msgs
        ## left params
        self.pwm_left_out =  Int16()
        self.pwm_left_out.data = 30     ## for now insert here to set velocity
        self.current_left_encoder_value = 0
        self.last_left_encoder_value = 0
        
        self.lwheel_rpm = Float64()
        self.lwheel_rpm.data = 0

        ## right params
        self.pwm_right_out =  Int16()
        self.pwm_right_out.data = 30      ## for now insert here to set velocity
        self.last_right_encoder_value = 0
        self.current_right_encoder_value = 0
        self.rwheel_rpm = Float64()
        self.rwheel_rpm.data = 0


        # init parameters
        self.R = 0.127
        self.L = 0.4
        self.N = 480
        self.x = 0  
        self.y = 0 
        self.theta = 0

        ## init odom and tf
        self.odom = Odometry()
        self.odom_broadcaster = TransformBroadcaster()

        self.update_pose()


    def control_vel_callback(self,msg):
        ## calculate cmd_rpms from a given cmd_vel and insert to the pid controller for pwm output


        #self.pwm_left_out.data =  palPID(.....)
        #self.pwm_right_out.data = palPID(.....)
        pass

    def publish(self):
        
        # publish rpm
        self.left_wheel_rpm_pub.publish(self.lwheel_rpm)
        self.right_wheel_rpm_pub.publish(self.rwheel_rpm)

        # publish pwm
        self.left_pwm_publisher.publish(self.pwm_left_out)
        self.right_pwm_publisher.publish(self.pwm_right_out)

        # publish odom
        self.odom_publisher.publish(self.odom)

    def leftEncoder_callback(self,msg):
        self.current_left_encoder_value = msg.data
    
    def rightEncoder_callback(self,msg):
        self.current_right_encoder_value = msg.data

    def calc_ticks(self,current_ticks , last_ticks):
        if((current_ticks>0)and(last_ticks<0)):
            return ((current_ticks - self.encoder_maximum)+(self.encoder_minimum-last_ticks))
        elif((current_ticks<0)and(last_ticks>0)):
            return ((current_ticks - self.encoder_minimum)+(self.encoder_maximum-last_ticks))
        else:
            return current_ticks - last_ticks


    def update_pose(self):
        
        while not rospy.is_shutdown():

            current_lticks = self.current_left_encoder_value
            last_lticks = self.last_left_encoder_value
            delta_lticks = self.calc_ticks(current_lticks,last_lticks)

            current_rticks = self.current_right_encoder_value
            last_rticks = self.last_right_encoder_value
            delta_rticks =  self.calc_ticks(current_rticks,last_rticks)
            rospy.loginfo("right delta ticks [ticks] = %s" , delta_rticks)

            dl = 2 * pi * self.R * (delta_lticks/self.N)
            dr = 2 * pi * self.R * (delta_rticks/self.N)
            rospy.loginfo("right wheel distance [m/s] = %s" ,dr)

            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            rospy.loginfo("dt [s] = %s" , dt)

            self.lwheel_rpm.data =  (delta_lticks/self.N)*(60/dt)
            self.rwheel_rpm.data = (delta_rticks/self.N)*(60/dt)
            rospy.loginfo("right rpm [rev/min] = %s" , self.rwheel_rpm.data)

            vl = dl/dt
            vr = dr/dt       
            v =  abs((vl + vr)/2)
            rospy.loginfo("linear velocity [m/s] = %s" , v)
            w = (vr - vl)/self.L
            rospy.loginfo("angular velocity [deg/s] = %s" , w)

            delta_x = v * cos(self.theta)
            delta_y = v * sin(self.theta)
            delta_theta = w

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            if (self.theta > (180)):
                self.theta = self.theta - 180
            if (self.theta < -180 ):
                self.theta = self.theta + 180

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

            self.publish()
            self.rate.sleep()



    def odom_msg_init(self,v,w,odom_quat):
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        # set the position and orientation
        self.odom.pose.pose = Pose(Point(self.x,self.y,0) , Quaternion(*odom_quat))
        # set velocity
        self.odom.twist.twist = Twist(Vector3(v,0,0), Vector3(0,0,w))
        
    


    def stop_robot():
        return            

if __name__ == '__main__':
    rospy.init_node('pal_controller_node', anonymous=True)
    
    pal_control = Controller()
   
    
    rospy.spin()
    
    #rate = rospy.Rate(100)
    
    #while not rospy.is_shutdown():
#
    #    
    #    pal_control.update_pose()
    #    
    #    pal_control.publish()
    #    
    #    rate.sleep()

    
    
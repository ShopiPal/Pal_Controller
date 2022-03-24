/*
 
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */
 
#include <ros.h>
#include <std_msgs/Int16.h>

 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18 //yellow
#define ENC_IN_RIGHT_A 3 //yellow
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19 // white
#define ENC_IN_RIGHT_B 2 // white
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("/encoder_right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("/encoder_left_ticks", &left_wheel_tick_count);
 
// Time interval for measurements in milliseconds
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
int pwmLeftReq = 0;
int pwmRightReq = 0;


 
// Motor A connections -------------- TO DO --------------
const int pwmA = 12 ; //white
const int in1 = 11 ; // orange
const int in2 = 10 ; // purple
  
// Motor B connections
const int pwmB = 6; //white 
const int in3 = 5; //orange
const int in4 = 4; //purple
 
// Record the time that the last velocity command was received
double left_lastPwmReceived = 0;
double right_lastPwmReceived = 0; 

/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
/////////////////////// Pwm subscribing ////////////////////////////

void set_pwm_right(const std_msgs::Int16& right_pwm_out){
  right_lastPwmReceived = (millis()/1000);
  pwmRightReq = right_pwm_out.data;
  set_pwm_values(pwmRightReq, pwmB , in3 , in4);
}
 
void set_pwm_left(const std_msgs::Int16& left_pwm_out){
  left_lastPwmReceived = (millis()/1000);
  pwmLeftReq = left_pwm_out.data;
  set_pwm_values(pwmLeftReq, pwmA , in1 , in2);
}
 
void set_pwm_values(int pwm_val ,int pwm_pin ,int in1_pin ,int in2_pin) {
 if(pwm_val >= 0 ){
    digitalWrite(in1_pin, LOW); // setting direction
    digitalWrite(in2_pin, HIGH);
    analogWrite(pwm_pin, pwm_val);
 }else {
    pwm_val = -1*pwm_val;
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW );
    analogWrite(pwm_pin, pwm_val);
 }
}


// Set up ROS subscriber to the pwm command
ros::Subscriber<std_msgs::Int16> subLeftPwm("/left_motor_pwm" , &set_pwm_left);
ros::Subscriber<std_msgs::Int16> subRightPwm("/right_motor_pwm" , &set_pwm_right);



void setup() {
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  // Motor control pins setup
  // left motor pin setup
  pinMode(pwmA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // right motor pin setup
  pinMode(pwmB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  analogWrite(pwmA, 0); 
  analogWrite(pwmB, 0);
 
  // ROS Setup
  //nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subLeftPwm);
  nh.subscribe(subRightPwm);
}

void loop() {
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and set pwm
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
 
     
  }
   
  // Stop the car if there are no pwm messages in the last 1 sec
  if((millis()/1000) - left_lastPwmReceived > 1) {
    pwmLeftReq = 0;
    analogWrite(pwmA, 0);
  }
  if((millis()/1000) - right_lastPwmReceived > 1) {
    pwmRightReq = 0;
    analogWrite(pwmB, 0);
  }
  
}

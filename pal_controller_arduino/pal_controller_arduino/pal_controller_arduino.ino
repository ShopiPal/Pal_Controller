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
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <sensor_msgs/Range.h>

//velocity calculation
#include <util/atomic.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// sonar Publishing Variables and Constants ///////////////
#define SONAR_NUM 6          //The number of sensors. // update to 6 
#define MAX_DISTANCE 250     //Max distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
unsigned long _timerStart = 0;
 
int LOOPING = 10 ; //Loop for every 10 milliseconds.
 
uint8_t oldSensorReading[SONAR_NUM];    //Store last valid value of the sensors.

uint8_t frontSensor;             //Store raw sensor's value.
uint8_t backSensor;
uint8_t rightSensor;
uint8_t leftSensor;
uint8_t front_rightSensor;
uint8_t front_leftSensor;
 
uint8_t frontSensorKalman;       //Store filtered sensor's value.
uint8_t backSensorKalman;
uint8_t rightSensorKalman;
uint8_t leftSensorKalman;
uint8_t front_rightSensorKalman;
uint8_t front_leftSensorKalman;
 
NewPing sonar[SONAR_NUM] = {
  NewPing(48, 49, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping. // front
  NewPing(46, 47, MAX_DISTANCE), // back
  NewPing(44, 45, MAX_DISTANCE), // right
  NewPing(42, 43, MAX_DISTANCE), // left 
  NewPing(40, 41, MAX_DISTANCE), // front_right
  NewPing(38, 39, MAX_DISTANCE) // front_left
};
 
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_front(1, 1, 0.01);
SimpleKalmanFilter KF_back(1, 1, 0.01);
SimpleKalmanFilter KF_right(1, 1, 0.01);
SimpleKalmanFilter KF_left(1, 1, 0.01);
SimpleKalmanFilter KF_front_right(1, 1, 0.01);
SimpleKalmanFilter KF_front_left(1, 1, 0.01);

//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}
 
// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
//Return the last valid value from the sensor.
void oneSensorCycle() {
  frontSensor       = returnLastValidRead(0, cm[0]);
  backSensor        = returnLastValidRead(1, cm[1]);
  rightSensor       = returnLastValidRead(2, cm[2]);
  leftSensor        = returnLastValidRead(3, cm[3]);
  front_rightSensor = returnLastValidRead(4, cm[4]);
  front_leftSensor  = returnLastValidRead(5, cm[5]);
  
}
 
//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}
 
//Apply Kalman Filter to sensor reading.
void applyKF() {
   frontSensorKalman        = KF_front.updateEstimate(frontSensor);       
   backSensorKalman         = KF_back.updateEstimate(backSensor);    
   rightSensorKalman        = KF_right.updateEstimate(rightSensor);
   leftSensorKalman         = KF_left.updateEstimate(leftSensor);
   front_rightSensorKalman  = KF_front_right.updateEstimate(front_rightSensor);
   front_leftSensorKalman   = KF_front_left.updateEstimate(front_leftSensor);  
}
 
void startTimer() {
  _timerStart = millis();
}
 
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
 
void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.5;
}
 
//Create instances for range messages.
sensor_msgs::Range range_front;
sensor_msgs::Range range_back;
sensor_msgs::Range range_right;
sensor_msgs::Range range_left;
sensor_msgs::Range range_front_right;
sensor_msgs::Range range_front_left;
 
//Create publisher objects for all sensors
ros::Publisher pub_range_front("/sonar_front", &range_front);
ros::Publisher pub_range_back("/sonar_back", &range_back);
ros::Publisher pub_range_right("/sonar_right", &range_right);
ros::Publisher pub_range_left("/sonar_left", &range_left);
ros::Publisher pub_range_front_right("/sonar_front_right", &range_front_right);
ros::Publisher pub_range_front_left("/sonar_front_left", &range_front_left);


////////////////// Encoder & Velocities variables and consts /////////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18 //white
#define ENC_IN_RIGHT_A 2 //white
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19 // yellow
#define ENC_IN_RIGHT_B 3 // yellow 
 
//geometric params
const int N = 480; // <<<check
const float R = 0.125/2;

// Encoders 
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i_left = 0;
volatile long prevT_i_left = 0;
int posPrev_left = 0;

volatile int pos_i_right = 0;
volatile long prevT_i_right = 0;
int posPrev_right = 0;

// velocities
float vr_prev_raw = 0;
float vr_curr_filter = 0;

float vl_prev_raw = 0;
float vl_curr_filter = 0;

// Time interval for measurements in milliseconds
long previousMillis = 0;
long currentMillis = 0;
// previous time in micros << check if necessary
long prevT = 0;

////////// Encoders & velocities & distance ROS topics //////

// right:
// encoder:
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("/encoder_right_ticks", &right_wheel_tick_count);

// velocity:
// publisher of velocity filter data
std_msgs::Float32 vr_current_filter; 
ros::Publisher vr_current_filter_Pub("/velocity/vr_current/filter",&vr_current_filter); 
// publisher of velocity raw data
std_msgs::Float32 vr_current_raw; 
ros::Publisher vr_current_raw_Pub("/velocity/vr_current/raw",&vr_current_raw); 

// left:
// encoder:
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("/encoder_left_ticks", &left_wheel_tick_count);
 
// velocity:
// publisher of velocity filter data
std_msgs::Float32 vl_current_filter; 
ros::Publisher vl_current_filter_Pub("/velocity/vl_current/filter",&vl_current_filter); 
// publisher of velocity raw data
std_msgs::Float32 vl_current_raw; 
ros::Publisher vl_current_raw_Pub("/velocity/vl_current/raw",&vl_current_raw); 


///// topic total distance of the robot

std_msgs::Float32 distance; 
ros::Publisher distance_Pub("/distance/dc",&distance);

/////////////////////// encoders handle functions ////////////////
// left:
void readEncoder_left(){
  // Read encoder B when ENCA rises
  int b_left = digitalRead(ENC_IN_LEFT_B);
  int increment_left = 0;
  if(b_left>0){
    // If B is high, increment backward
    increment_left = -1;
  }
  else{
    // Otherwise, increment forward
    increment_left = 1;
  }
  pos_i_left = pos_i_left + increment_left;
}

// right:
void readEncoder_right(){
  // Read encoder B when ENCA rises
  int b_right = digitalRead(ENC_IN_RIGHT_B);
  int increment_right = 0;
  if(b_right>0){
    // If B is high, increment forward
    increment_right = 1;
  }
  else{
    // Otherwise, increment backward 
    increment_right = -1;
  }
  pos_i_right = pos_i_right + increment_right;
}




////////////////// Motor Controller Variables and Constants ///////////////////
int pwmLeftReq = 0;
int pwmRightReq = 0;

// Motor A connections 
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


/////////////////////// setup ////////////////////////////

void setup() {

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  
  // init Range msg
  sensor_msg_init(range_front,"/sonar_front");
  sensor_msg_init(range_back, "/sonar_back");
  sensor_msg_init(range_right,"/sonar_right");
  sensor_msg_init(range_left, "/sonar_left");
  sensor_msg_init(range_front_right,"/sonar_front_right");
  sensor_msg_init(range_front_left, "/sonar_front_left");

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT); 
  
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A),readEncoder_left,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A),readEncoder_right,RISING);

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
  nh.getHardware()->setBaud(115200); ///changed from 57600
  nh.initNode();

  nh.advertise(rightPub);
  nh.advertise(vr_current_filter_Pub);
  nh.advertise(vr_current_raw_Pub);

  nh.advertise(leftPub);
  nh.advertise(vl_current_filter_Pub);
  nh.advertise(vl_current_raw_Pub);

  nh.advertise(distance_Pub);
  nh.subscribe(subLeftPwm);
  nh.subscribe(subRightPwm);
  
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_back);
  nh.advertise(pub_range_right);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_front_right);
  nh.advertise(pub_range_front_left);
}

void loop() {

  unsigned long currentMillis = millis();
  
  if (currentMillis-previousMillis >= 33){
  // read the position in an atomic block
  // to avoid potential misreads
    int pos_right = 0;
    int pos_left = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      pos_left = pos_i_left;
      pos_right = pos_i_right;
    }
    
  // Compute velocity with method 1
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1.0e6; // [seconds]
    
    float vr_count = (pos_right - posPrev_right)/deltaT; // [count/sec]
    posPrev_right = pos_right; 
    float vl_count = (pos_left - posPrev_left)/deltaT; // [count/sec]
    posPrev_left = pos_left; 

    prevT = currT;

  // Convert count/s to RPM
    float vl_rpm = vl_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])
    float vr_rpm = vr_count/N*60.0; // [count/sec]/([count/rev]*[min/sec])

  // Convert RPM to Linear speed of the wheels
    float vl_curr_raw = (vl_rpm * 2 * PI * R) / 60   ; // ([Rev/min]*[rad/Rev]*[meters])/[sec/min]
    float vr_curr_raw = (vr_rpm * 2 * PI * R) / 60   ; // ([Rev/min]*[rad/Rev]*[meters])/[sec/min]
  
  // Low-pass filter (25 Hz cutoff) <<< check values
    vl_curr_filter = 0.854*vl_curr_filter + 0.0728*vl_curr_raw + 0.0728*vl_prev_raw;
    vl_prev_raw = vl_curr_raw;

    vr_curr_filter = 0.854*vr_curr_filter + 0.0728*vr_curr_raw + 0.0728*vr_prev_raw;
    vr_prev_raw = vr_curr_raw;
    
  
  // calc distances
    float dr = vr_curr_filter*deltaT ; // right wheel distance passed [meter/sec]*[sec] 
    float dl = vl_curr_filter*deltaT ; // left wheel distance passed [meter/sec]*[sec]
    float dc = (dr + dl)/2 ; // total distance robot passed [meter]

  //if (isTimeForLoop(LOOPING)) {      // ---> need to check
    sensorCycle();
    oneSensorCycle();
    applyKF(); //with filtering
    range_front.range   = frontSensorKalman; 
    range_back.range = backSensorKalman;
    range_right.range  = rightSensorKalman;
    range_left.range  = leftSensorKalman;
    range_front_right.range  = front_rightSensorKalman;
    range_front_left.range  = front_leftSensorKalman;
    
    // without the filter
    //range_front.range   = frontSensor; 
    //range_back.range = backSensor;
    //range_right.range  = rightSensor;
    //range_left.range  = leftSensor;
    //range_front_right.range  = front_rightSensor;
    //range_front_left.range  = front_leftSensor;
        
    
    //////////// set ROS msgs //////////////

    // set encoder msgs [counts]
    left_wheel_tick_count.data = pos_left ; 
    right_wheel_tick_count.data = pos_right ; 

    // set velocities msgs [m/s]
    // raw
    vl_current_raw.data = vl_curr_raw;
    vr_current_raw.data = vr_curr_raw;
    
    // filter
    vr_current_filter.data = vr_curr_filter;
    vl_current_filter.data = vl_curr_filter;
    

    // set distance msgs [meter]
    distance.data = dc ;
      
     // Stop the car if there are no pwm messages in the last 3 sec
    if((millis()/1000) - left_lastPwmReceived > 3) {   ////// changed to 3 from 1 sec
      pwmLeftReq = 0;
      analogWrite(pwmA, 0);
    }
    if((millis()/1000) - right_lastPwmReceived > 3) {   ////// changed to 3 from 1 sec
      pwmRightReq = 0;
      analogWrite(pwmB, 0);
    }
    
    //////////  publishing to topics ///////

    range_front.header.stamp = nh.now();
    range_back.header.stamp = nh.now();
    range_right.header.stamp = nh.now();
    range_left.header.stamp = nh.now();
    range_front_right.header.stamp = nh.now();
    range_front_left.header.stamp = nh.now();
  
    pub_range_front.publish(&range_front);
    pub_range_back.publish(&range_back);
    pub_range_right.publish(&range_right);
    pub_range_left.publish(&range_left);
    pub_range_front_right.publish(&range_front_right);
    pub_range_front_left.publish(&range_front_left);
    
    
    leftPub.publish( &left_wheel_tick_count ); 
    rightPub.publish( &right_wheel_tick_count );
    
    vr_current_raw_Pub.publish( &vr_current_raw);
    vr_current_filter_Pub.publish( &vr_current_filter);
    vl_current_raw_Pub.publish( &vl_current_raw);
    vl_current_filter_Pub.publish( &vl_current_filter);
    
    distance_Pub.publish( &distance);
    
    
    // update previous time
    //startTimer();
    previousMillis = currentMillis;
  } 
 nh.spinOnce(); 
}

#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
 
#define SONAR_NUM 6          //The number of sensors. // update to 6 
#define MAX_DISTANCE 200     //Max distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
 
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
unsigned long _timerStart = 0;
 
int LOOPING = 40; //Loop for every 40 milliseconds.
 
uint8_t oldSensorReading[3];    //Store last valid value of the sensors.
 
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
SimpleKalmanFilter KF_front(2, 2, 0.01);
SimpleKalmanFilter KF_back(2, 2, 0.01);
SimpleKalmanFilter KF_right(2, 2, 0.01);
SimpleKalmanFilter KF_left(2, 2, 0.01);
SimpleKalmanFilter KF_front_right(2, 2, 0.01);
SimpleKalmanFilter KF_front_left(2, 2, 0.01);


ros::NodeHandle nh; //create an object which represents the ROS node.
 
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
  leftSensor        = returnLastValidRead(3, cm[2]);
  front_rightSensor = returnLastValidRead(4, cm[2]);
  front_leftSensor  = returnLastValidRead(5, cm[2]);
  
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
  range_name.max_range = 2.0;
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

void setup() {
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  nh.initNode();
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_back);
  nh.advertise(pub_range_right);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_front_right);
  nh.advertise(pub_front_range_left);
 
  sensor_msg_init(range_front,"/sonar_front");
  sensor_msg_init(range_back, "/sonar_back");
  sensor_msg_init(range_right,"/sonar_right");
  sensor_msg_init(range_left, "/sonar_left");
  sensor_msg_init(range_front_right,"/sonar_front_right");
  sensor_msg_init(range_front_left, "/sonar_front_left");
}
 
void loop() {
  if (isTimeForLoop(LOOPING)) {
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
   // range_front.range   = frontSensor; 
   // range_back.range = backSensor;
   // range_right.range  = rightSensor;
   // range_left.range  = leftSensor;
   // range_front_right.range  = front_rightSensor;
   // range_front_left.range  = front_leftSensor;
    
    
    
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
    pub_front_range_left.publish(&range_front_left);
 
    startTimer();
  }
  nh.spinOnce();//Handle ROS events
}

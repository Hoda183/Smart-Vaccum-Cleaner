#include <ros.h>
#include <std_msgs/Float32.h>

//initialize ROS NodeHandle
ros::NodeHandle nh;

//create messages for Ultrasonic
std_msgs::Float32 speed_msg;

//create publishers for Ultrasonic
ros::Publisher speed_pub("wheel_speed", &speed_msg);

// define pins and variables
const int irSensorPin = 2; 
volatile int line_count = 0; 
unsigned long last_time_speed = 0; 


//IR Sensor Function
void detectLine() {
 
  line_count++;
}

void setup() {
  //initialize the node
  nh.initNode();
  // advertise topics
  nh.advertise(speed_pub);
  
  pinMode(irSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(irSensorPin), detectLine, RISING);

  }

void loop() {
  unsigned long current_time = millis();

  
  if (current_time - last_time_speed >= 1000) { 
    float revs = line_count / 4.0; 
    float speed = (revs / ((current_time - last_time_speed) / 1000.0))*60; //speed rpm
    
    speed_msg.data = speed;
    speed_pub.publish(&speed_msg); //puplish the message on the topic"speed_pup"

    
    line_count = 0;
    last_time_speed = current_time;
  }

  
  nh.spinOnce();
  delay(100);
}

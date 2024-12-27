#include <ros.h>
#include <std_msgs/Int16.h>

//initialize ROS NodeHandle
ros::NodeHandle nh;

//create messages for Ultrasonic
std_msgs::Int16 ultrasonic_msg;

//create publishers for Ultrasonic
ros::Publisher pub_ultrasonic("ultrasonic/range", &ultrasonic_msg);

// define pins
const byte TRIG_PIN = 9; 
const byte ECHO_PIN = 10;


void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
 
  //initialize the node
  nh.initNode();
  
  // advertise topics
  nh.advertise(pub_ultrasonic);
  
}

//function of ultrasonic
int readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int duration = pulseIn(ECHO_PIN, HIGH);
  return (duration * 0.034 / 2) ; // Convert to cm
}


void loop() {
  // Read ultrasonic distance
  ultrasonic_msg.data = readUltrasonicDistance(); 
  pub_ultrasonic.publish(&ultrasonic_msg); //puplish the message on the topic"pup_ultrasonic"

  nh.spinOnce();
  delay(100);
}

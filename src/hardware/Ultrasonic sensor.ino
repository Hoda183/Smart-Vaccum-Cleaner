#include <ros.h>
#include <sensor_msgs/Range.h>

//initialize ROS NodeHandle
ros::NodeHandle nh;

//create messages for Ultrasonic
sensor_msgs::Range ultrasonic_msg;

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
  
  // configure ultrasonic sensor message
  ultrasonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonic_msg.field_of_view = 0.5; //angle of view "28"
  ultrasonic_msg.min_range = 2.0;  // in cm
  ultrasonic_msg.max_range = 400.0;   // in cm
}

//function of ultrasonic
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration * 0.034 / 2) ; // Convert to cm
}


void loop() {
  // Read ultrasonic distance
  float distance = readUltrasonicDistance();
  //check that the distance in the range if in range return the value if not return the 400
  ultrasonic_msg.range = (distance >= ultrasonic_msg.min_range && distance <= ultrasonic_msg.max_range) ? distance : ultrasonic_msg.max_range;
  ultrasonic_msg.header.stamp = nh.now(); //update time
  pub_ultrasonic.publish(&ultrasonic_msg); //puplish the message on the topic"pup_ultrasonic"

  nh.spinOnce();
  delay(100);
}

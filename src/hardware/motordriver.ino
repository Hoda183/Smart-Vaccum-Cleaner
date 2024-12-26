#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

// Motor pin definitions
#define IN1 2
#define IN2 4
#define EN1 3
#define IN3 10
#define IN4 11
#define EN2 9

// ROS node handle
ros::NodeHandle nh;

// Variable to store motor speeds
int left_motor_speed = 0;
int right_motor_speed = 0;

// ROS subscriber callback
void motorSpeedCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data.size() == 2) { // Ensure two values are received
    left_motor_speed = msg.data[0];
    right_motor_speed = msg.data[1];
    controlMotors(left_motor_speed, right_motor_speed); // Update motor speeds
  }
}

// Initialize subscriber
ros::Subscriber<std_msgs::Int32MultiArray> motorSpeedSub("motor_speeds", motorSpeedCallback);

void setup() {
  // Set motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize ROS node
  nh.initNode();

  // Subscribe to the 'motor_speeds' topic
  nh.subscribe(motorSpeedSub);
}

void loop() {
  // Process incoming ROS messages
  nh.spinOnce();

  // Small delay for stability
  delay(10);
}

// Function to control motor speeds
void controlMotors(int left_speed, int right_speed) {
  // Set motor directions and speeds based on input
  if (left_speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    left_speed = -left_speed; // Convert to positive for PWM
  }

  if (right_speed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    right_speed = -right_speed; // Convert to positive for PWM
  }

  // Write speeds to the motor driver
  analogWrite(EN1, constrain(left_speed, 0, 255));
  analogWrite(EN2, constrain(right_speed, 0, 255));

}

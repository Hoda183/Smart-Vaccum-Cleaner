#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>

// Wifi
#include <WiFi.h>  // For ESP32 Wi-Fi library
#include <geometry_msgs/Twist.h>



// Ultrasonic
#include <std_msgs/Int16.h>
#define  TRIG_PIN 4
#define ECHO_PIN 5



// Motor Driver
#include <std_msgs/Int32MultiArray.h>

// Motor pin definitions
#define EN1 12
#define EN2 13

#define IN1 18
#define IN2 19

#define IN3 21
#define IN4 22






















unsigned long previousMillis = 0; // Stores the last time the loop ran
const unsigned long interval = 100; // Interval in milliseconds



const char* ssid     = "zeroMachine";  // Replace with your Wi-Fi SSID
const char* password = "helloworld";  // Replace with your Wi-Fi password
// Set the rosserial socket server IP address (ROS master IP)
IPAddress server(10, 42, 0, 1);  // Replace with your ROS Master's IP

// const char* ssid     = "Home";  // Replace with your Wi-Fi SSID
// const char* password = "notwelcome0000";  // Replace with your Wi-Fi password
// IPAddress server(192, 168, 1, 20);  // Replace with your ROS Master's IP

// Set the rosserial socket server port (default: 11411)
const uint16_t serverPort = 11411;

ros::NodeHandle nh;  // Create ROS node handle


//create messages for Ultrasonic
std_msgs::Int16 ultrasonic_msg;

//create publishers for Ultrasonic
ros::Publisher pub_ultrasonic("ultrasonic/range", &ultrasonic_msg);



// Variable to store motor speeds
int left_motor_speed = 0;
int right_motor_speed = 0;


// ROS subscriber callback
void motorSpeedCallback(const std_msgs::Int32MultiArray& msg) {
  // if (msg.data.size() == 2) { // Ensure two values are received
    if (msg.data_length == 2) { // Ensure two values are received
    left_motor_speed = msg.data[0];
    right_motor_speed = msg.data[1];
    controlMotors(left_motor_speed, right_motor_speed); // Update motor speeds
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> motorSpeedSub("motors_speed", &motorSpeedCallback);

WiFiClient wifiClient;  // Wi-Fi client for socket connection



void setup() {
  Serial.begin(115200);  // Start serial communication for debugging

  // ultrasonic configs
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // Motor Driver configs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);



  // WIFI configs
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  // Display ESP32's IP address

  // Establish socket connection with ROS master using WiFiClient
  if (wifiClient.connect(server, serverPort)) {

    // Use rosserial's hardware interface with WiFiClient
    nh.getHardware()->setConnection(server, serverPort);  // Pass WiFiClient here
    nh.initNode();  // Initialize ROS node



    // advertise topics
    nh.advertise(pub_ultrasonic);




    // motor driver
    nh.subscribe(motorSpeedSub);

  } else {
    Serial.println("Connection to ROS Master failed");
  }

  Serial.print("ESP32 IP = ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (nh.connected()) {
    unsigned long currentMillis = millis(); // Get the current time
    // ultrasonic_msg.data = readUltrasonicDistance(); 
    // pub_ultrasonic.publish(&ultrasonic_msg); //puplish the message on the topic"pup_ultrasonic
    if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Update the time

    // Read ultrasonic distance
    ultrasonic_msg.data = readUltrasonicDistance(); 
    // ultrasonic_msg.data = 1; 
    pub_ultrasonic.publish(&ultrasonic_msg); // Publish the message on the topic "pub_ultrasonic"

  }


  } else {
    Serial.println("Not connected to ROS Master");
  }

  nh.spinOnce();  // Call rosserial processing
  

  // Check if the interval has passed
  
  // delay(10); 
}


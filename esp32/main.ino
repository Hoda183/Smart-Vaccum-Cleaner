#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>

// Wifi
#include <WiFi.h>  // For ESP32 Wi-Fi library



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

// switch 
#include <std_msgs/Bool.h>
#define GPIO_PIN 25



unsigned long previousMillis = 0; // Stores the last time the loop ran
const unsigned long interval = 100; // Interval in milliseconds



const char* ssid     = "zeroMachine";  
const char* password = "helloworld";  
// Set the rosserial socket server IP address (ROS master IP)
IPAddress server(10, 42, 0, 1); 


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
    if (msg.data_length == 2) { // Ensure two values are received
    left_motor_speed = msg.data[0];
    right_motor_speed = msg.data[1];
    controlMotors(left_motor_speed, right_motor_speed); // Update motor speeds
  }
}


// Callback function for the 'vaccum_state' topic
void vaccumStateCallback(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(GPIO_PIN, HIGH); // Turn GPIO 25 ON
    }
  else {
    digitalWrite(GPIO_PIN, LOW); // Turn GPIO 25 OFF
    
     }
}

// Subscriber for the 'vaccum_state' topic
ros::Subscriber<std_msgs::Bool> vaccumStateSub("vaccum_state", vaccumStateCallback);


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

  // Switch configs
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, LOW); // Default state OFF


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
    nh.getHardware()->setConnection(server, serverPort); 
    nh.initNode();



    // advertise topics
    nh.advertise(pub_ultrasonic);

    // Subscribe to the 'vaccum_state' topic
    nh.subscribe(vaccumStateSub);


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
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read ultrasonic distance
    ultrasonic_msg.data = readUltrasonicDistance(); 

    // Publish the message on the topic "pub_ultrasonic"
    pub_ultrasonic.publish(&ultrasonic_msg); 

  }


  } else {
    Serial.println("Not connected to ROS Master");
  }

  nh.spinOnce();  // Call rosserial processing

}

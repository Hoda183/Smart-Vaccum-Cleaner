# Smart-Vacuum-Cleaner
![Preview Image](https://github.com/user-attachments/assets/9cf2ec3e-eb46-42fa-8bce-25d9f6d64aed)


This project demonsterates the use of ROS to build and control an autonomous vacuum cleaner capable of avoiding obstacles dynamically. Below is an overview of the ROS nodes and their functionality:

## Installation

1. start the catkin workspace by `catkin_make`
2. install rosserial
    
    ```
    sudo apt update
    sudo apt install ros-<ros-distro>-rosserial
    sudo apt install ros-<ros-distro>-rosserial-arduino
    ```
    
3. install rosbridge
    
    ```
    sudo apt update
    sudo apt install ros-noetic-rosbridge-server
    ```
    

## Setup

1. upload [main.ino](https://github.com/Hoda183/Smart-Vaccum-Cleaner/blob/main/esp32/main.ino) to esp32 board
2. run ROS master node
    
    ```
    roscore
    ```
    
3. run roslaunch file
on this step it will start :
    
    ```
    roslaunch controller/launch/start.launch
    ```
    
    - rosserial server (for comunnicating between ROS master and ESP32)
    - rosbridge_websocket on port 8080 (for communcating between ROS master and web application)
    - control_mode server (used to control the mode of the robot either manual control or autonomous)
4. open localhost link on port 8080

## Hardware

## Mechanical Chasis
Front             | Back
:-------------------------:|:-------------------------:
![Front](https://github.com/user-attachments/assets/e06aa421-10a1-4949-b85c-96df5c17f2cb) | ![Back](https://github.com/user-attachments/assets/176de848-481c-4e12-af8c-0ca374f35ffa)
Side View | Bottom 
![Side View](https://github.com/user-attachments/assets/bcec10f7-154e-4c1d-a9f9-1fe990e9b0af) | ![Bottom](https://github.com/user-attachments/assets/541111e6-499d-41dd-9e43-2ef7bf6bb94f)



### Credits
we started with this [Arduino Vacuum Cleaner Robot Design
](https://grabcad.com/library/arduino-vacuum-cleaner-robot-design-1) by [
Mohammed Ismail](https://grabcad.com/mohammed.ismail-48)
### Problems we faced
1. The fan suction was weak so we made few changes to the design:
    * changing the motor to 5v dc motor with blades
    * changing the dimensios of the suction box to fit the new design
    * adding a megnatic cover to bottom of the box to be able to empty the box
2. after changing the dimension and the position of the box the center of mass was shifted to +ve y-axis of the robot 
    * to solve this we added a weight almost 750gm between the wheels to get back the center of mass to the center but it could be enhanced by changing the 3d design 

## Web App
Main Menu             | Keyboard Menu
:-------------------------:|:-------------------------:
![WhatsApp Image 2025-01-04 at 01 16 39_38535d65](https://github.com/user-attachments/assets/b12a014c-b660-40fa-9b47-e96175563901) | ![WhatsApp Image 2025-01-04 at 01 16 39_bbc368ed](https://github.com/user-attachments/assets/c1b5f386-8ca0-42b8-87f8-bab2e7a4c0e7) *only works on mobile screens

### Functions:
1. **Keyboard** : this mode the robot will only move using the keyboard of the host of ros master or in the phone interface using [motors_test script](https://github.com/Hoda183/Smart-Vaccum-Cleaner/blob/main/src/controller/scripts/motors_test.py)
2. **Ps3** : this mode will make the robot moves with ps3 controller which will be connected to host of ros master node using [ps3_motors script](https://github.com/Hoda183/Smart-Vaccum-Cleaner/blob/main/src/controller/scripts/ps3_motors.py)
3. **Autonomus** : thos mode will make the robot moves only using the control signal from the [controller node](https://github.com/Hoda183/Smart-Vaccum-Cleaner/blob/main/src/controller/scripts/random_motion.py) without any manual control
4. **Stop** : this will force the robot to stop by disable all controlling nodes and will send a value of [0, 0] to motors_speed topic to make it stop
5. **Vaccum Mode** : This mode has a boolean value that will pass to either turn on or off the 5v dc motor that responsible for suction
## Features

1. **Obstacle Detection using Ultrasonic Sensor**
    - **Node:**
    'ultrasonic_sensor_node'
    - **functionality:**
        - Continuously reads the distance between the sensor and the nearest obstacle or wall.
        - Publishes the readings to a dedicated ROS topic (ultrasonic/range).
    - **Purpose**
        - Enables the control system to monitor the environment in real_time.
2. **Control Node for Decision Making**
    - **Node:**
    'control_node'
    - **Functionality:**
        - Subscribes to the ultrasonic sensor topic (ultrasonic/range) to receive distance readings.
        - Monitors the distance values and determines if the robot is approaching an obstacle.
        - Sends a signal to the motor driver via a topic (/motor_control) to:
            - Slow down when an obstacle is detected.
            - Stop if the obstacle is too close.
    - **Purpose**
        - Manages the robot's movement based on real_time data from the sensors.
3. **Motor Driver Communication**
    - **Node:**
    'motor_driver_node'
    - **Functionality:**
        - Subscribes to the control topic (/motor_control) to receive speed and direction commands.
        - Adjusts the motor's speed and direction accordingly.
    - **Purpose**
        - Executes movement commands, ensuring smooth operation of the vacuum cleaner.

## Workflow

1. The **Ultrasonic sensor** continuously puplishes distance readings.
2. the **control node** monitors these readings and evaluates the proximity of obstacles.
3. Based on the obstacle's distance:
    - Commands to slow down or stop are sent to the **motor driver**.
4. The **motor driver node** adjusts the robot's movement accordingly.

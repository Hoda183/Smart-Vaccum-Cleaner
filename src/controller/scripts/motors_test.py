#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from pynput import keyboard
stop = False
lastOrder = 'back'
# Global variables to store PWM values for left and right motors
left_motor_pwm = 0
right_motor_pwm = 0

# Function to send PWM values to the motors_speed topic
def send_pwm_message(left_pwm, right_pwm):
    msg = Int32MultiArray()
    msg.data = [left_pwm, right_pwm]
    
    # Publish the message to the topic
    pub.publish(msg)
    rospy.loginfo(f"Published: Left PWM = {left_pwm}, Right PWM = {right_pwm}")

# Callback function for keyboard press
def on_press(key):
    global left_motor_pwm, right_motor_pwm, stop, lastOrder
    
    try:
        # Check which key was pressed
        if key == keyboard.Key.up:
            # Increase PWM for both motors (forward)
            if not stop and lastOrder == 'back':
                left_motor_pwm = 0
                right_motor_pwm = 0
                stop = True
            else:
                left_motor_pwm = 255
                right_motor_pwm = 255
                stop = False
                lastOrder = 'front'
            rospy.loginfo("Up Arrow pressed: Increasing PWM (Forward)")

        elif key == keyboard.Key.down:
            # Decrease PWM for both motors (backward)
            if not stop and lastOrder == 'front':
                left_motor_pwm = 0
                right_motor_pwm = 0
                stop = True
            else:
                left_motor_pwm = -255
                right_motor_pwm = -255
                stop = False
                lastOrder = 'back'
            rospy.loginfo("Down Arrow pressed: Decreasing PWM (Backward)")

        elif key == keyboard.Key.right:
            # Increase PWM for left motor, decrease for right motor (turn left)
            left_motor_pwm =0
            right_motor_pwm =255
            stop = False
            rospy.loginfo("Left Arrow pressed: Turning Left")


        elif key == keyboard.Key.left:
            # Increase PWM for right motor, decrease for left motor (turn right)
            left_motor_pwm = 255
            right_motor_pwm = 0
            stop = False
            rospy.loginfo("Right Arrow pressed: Turning Right")

        # Ensure PWM values are within range (0-255)
        # left_motor_pwm = max(-255, min(left_motor_pwm, 255))
        # right_motor_pwm = max(-255, min(right_motor_pwm, 255))

        # Publish PWM values to the motors_speed topic
        send_pwm_message(left_motor_pwm, right_motor_pwm)

    except Exception as e:
        rospy.logerr(f"Error handling key press: {e}")

# Initialize the ROS node
rospy.init_node('keyboard_to_pwm', anonymous=False)

# Create a publisher for the motors_speed topic
# Ensure this is done before using `pub`
pub = rospy.Publisher('motors_speed', Int32MultiArray, queue_size=10)

# Set up listener for keyboard events
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Keep the script running and wait for messages to be published
rospy.spin()

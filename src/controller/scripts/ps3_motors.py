#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

# Callback function to handle joystick input
def joy_callback(data):
    # Initialize both motor speeds to 0 (steady state is stop)
    left_motor_speed = 0
    right_motor_speed = 0

    # Left joystick X-axis controls steering (left/right)
    if data.axes[0] > 0:  # Joystick moved to the right
        left_motor_speed = 255  # Left motor moves forward
        right_motor_speed = 0  # Right motor stops
    elif data.axes[0] < 0:  # Joystick moved to the left
        left_motor_speed = 0  # Left motor stops
        right_motor_speed = 255  # Right motor moves forward
    else:  # Joystick is centered or neutral
        # Left joystick Y-axis controls forward/backward (up/down)
        if data.axes[1] > 0:  # Joystick moved up (forward)
            left_motor_speed = 255  # Both motors move forward
            right_motor_speed = 255
        elif data.axes[1] < 0:  # Joystick moved down (backward)
            left_motor_speed = -255  # Both motors move backward
            right_motor_speed = -255
        else:  # Joystick is in the neutral position (no movement)
            left_motor_speed = 0
            right_motor_speed = 0

    # Create the Int32MultiArray message to be published
    motors_speed_msg = Int32MultiArray()
    motors_speed_msg.data = [left_motor_speed, right_motor_speed]

    # Publish the motor speeds on the single topic
    motors_speed_pub.publish(motors_speed_msg)

    # Log the values for debugging
    rospy.loginfo(f"Motors speed - Left: {left_motor_speed}, Right: {right_motor_speed}")

def listener():
    # Initialize the ROS node
    rospy.init_node('ps3_motors', anonymous=False)

    # Create a publisher for motors_speed
    global motors_speed_pub
    motors_speed_pub = rospy.Publisher('/motors_speed', Int32MultiArray, queue_size=10)

    # Subscribe to the /joy topic to get joystick input
    rospy.Subscriber("/joy", Joy, joy_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()

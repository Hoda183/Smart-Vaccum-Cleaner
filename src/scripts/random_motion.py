#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray

velocity_msg = Int32MultiArray()
ultrasonic_read = 150
last_distance = 150
stable_readings_count = 0

WALL_THRESHOLD = 100
MAX_SPEED = 100
MIN_SPEED = 50
TURN_SPEED = 120
MIN_DISTANCE = 40
MOVING_BACK_TIME = 2
MIN_TURN_TIME = 0.75
MAX_TURN_TIME = 1.5

current_state = 'MOVING_FORWARD'
state_start_time = None
compare_one = 0
compare_two = 0

def ultrasonic_callback(data):
    global ultrasonic_read, last_distance, stable_readings_count

    if abs(data.data - last_distance) > 50:
        stable_readings_count = 0
        return

    stable_readings_count += 1

    if stable_readings_count >= 3:
        ultrasonic_read = data.data

    last_distance = data.data

def calculate_adaptive_speed():
    if ultrasonic_read > WALL_THRESHOLD:
        speed = MAX_SPEED
    elif MIN_DISTANCE <= ultrasonic_read <= WALL_THRESHOLD:
        speed = int(MAX_SPEED - (MAX_SPEED - MIN_SPEED) * (WALL_THRESHOLD - ultrasonic_read) / (WALL_THRESHOLD - MIN_DISTANCE))
    else:
        speed = 0
    return speed

def moving_back(current_time):
    global current_state, state_start_time, velocity_msg

    if state_start_time is None:
        state_start_time = current_time

    if current_time - state_start_time >= MOVING_BACK_TIME:
        current_state = 'DECISION'
        state_start_time = current_time
    else:
        velocity_msg.data = [-MAX_SPEED // 2, -MAX_SPEED // 2]

def decision_phase():
    global current_state, state_start_time, compare_one, compare_two, velocity_msg

    random_time = random.uniform(MIN_TURN_TIME, MAX_TURN_TIME)

    velocity_msg.data = [-TURN_SPEED, TURN_SPEED]
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < random_time:
        pup.publish(velocity_msg)
    compare_one = ultrasonic_read

    velocity_msg.data = [TURN_SPEED, -TURN_SPEED]
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < 2 * random_time:
        pup.publish(velocity_msg)
    compare_two = ultrasonic_read

    if compare_two > compare_one:
        current_state = 'MOVING_FORWARD'
    else:
        velocity_msg.data = [-TURN_SPEED, TURN_SPEED]
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2 * random_time:
            pup.publish(velocity_msg)

        current_state = 'MOVING_FORWARD'

def random_motion():
    global current_state, state_start_time
    rospy.init_node('random_motion', anonymous=True)
    global pup
    pup = rospy.Publisher('velocity', Int32MultiArray, queue_size=10)
    rospy.Subscriber('/ultrasonic/range', Int16, ultrasonic_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        current_time = rospy.get_time()

        if current_state == 'MOVING_FORWARD':
            speed = calculate_adaptive_speed()

            if speed == 0:
                current_state = 'MOVING_BACK'
                state_start_time = current_time
                velocity_msg.data = [-MAX_SPEED // 2, -MAX_SPEED // 2]
            else:
                velocity_msg.data = [speed, speed]

        elif current_state == 'MOVING_BACK':
            moving_back(current_time)

        elif current_state == 'DECISION':
            decision_phase()

        rospy.loginfo(f"Current state: {current_state}")
        pup.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        random_motion()
    except rospy.ROSInterruptException:
        pass

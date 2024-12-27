import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Constants
MAX_SPEED = 1
TURN_SPEED = 1
WALL_THRESHOLD = 5.0  # Example threshold for walls in the Turtlesim world
MIN_DISTANCE = 1.0
TURN_TIME = 2
MINI_FORWARD = 1.5

# Global variables
turtle_pose = Pose()
last_distance = float('inf')
zigzag_left = True
twist_msg = Twist()
current_state = 'MOVING_FORWARD'  # States: MOVING_FORWARD, TURNING_FIRST, MOVING_SHORT, TURNING_SECOND
coverage_count = 0
last_turn_time = 0
start_time = None

def pose_callback(data):
    """Update the turtle's current pose."""
    global turtle_pose
    turtle_pose = data


def calculate_distance_to_point(x, y):
    """Calculate distance from the turtle to a given point (x, y)."""
    return math.sqrt((turtle_pose.x - x)**2 + (turtle_pose.y - y)**2)

def calculate_adaptive_speed(distance):
    """Calculate speed with improved wall approach."""
    if distance > WALL_THRESHOLD:
        return MAX_SPEED
    elif MIN_DISTANCE <= distance <= WALL_THRESHOLD:
        # Smoother speed adjustment near walls
        normalized_distance = (distance - MIN_DISTANCE) / (WALL_THRESHOLD - MIN_DISTANCE)
        return MAX_SPEED * (0.4 + 0.6 * normalized_distance)
    return 0

def perform_turn_sequence(pub):
    """Execute improved turn sequence."""
    global current_state, zigzag_left, coverage_count, last_turn_time, twist_msg

    current_time = rospy.get_time()

    if current_state == 'MOVING_FORWARD':
        # Prepare for first turn
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        rospy.sleep(0.5)  # Complete stop
        current_state = 'TURNING_FIRST'
        last_turn_time = current_time

    elif current_state == 'TURNING_FIRST':
        # First turn
        twist_msg.linear.x = 0  # No forward motion
        twist_msg.angular.z = TURN_SPEED if zigzag_left else -TURN_SPEED

        if current_time - last_turn_time >= TURN_TIME:
            current_state = 'MOVING_SHORT'
            last_turn_time = current_time

    elif current_state == 'MOVING_SHORT':
        # Short forward movement
        twist_msg.linear.x = MAX_SPEED * 0.5
        twist_msg.angular.z = 0

        if current_time - last_turn_time >= MINI_FORWARD:
            current_state = 'TURNING_SECOND'
            last_turn_time = current_time

    elif current_state == 'TURNING_SECOND':
        # Second turn
        twist_msg.linear.x = 0  # No forward motion
        twist_msg.angular.z = TURN_SPEED if zigzag_left else -TURN_SPEED

        if current_time - last_turn_time >= TURN_TIME:
            current_state = 'MOVING_FORWARD'
            zigzag_left = not zigzag_left
            coverage_count += 1

    pub.publish(twist_msg)

def control_node():
    global twist_msg, current_state, start_time

    # Initialize ROS node
    rospy.init_node('coverage_controller', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # Publish to /turtle1/cmd_vel for Twist messages
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rate = rospy.Rate(10)
    start_time = rospy.get_time()

    # Define an example wall point (e.g., x = 10.0, y = turtle_pose.y)
    wall_x = 10.0
    while not rospy.is_shutdown():
        distance_to_wall = calculate_distance_to_point(wall_x, turtle_pose.y)

        if current_state == 'MOVING_FORWARD':
            speed = calculate_adaptive_speed(distance_to_wall)
            twist_msg.linear.x = speed
            twist_msg.angular.z = 0

            if speed == 0:  # Wall detected
                perform_turn_sequence(pub)
            else:
                pub.publish(twist_msg)
        else:
            perform_turn_sequence(pub)

        # Print some info for debugging
        elapsed_time = rospy.get_time() - start_time
        rospy.loginfo(f"State: {current_state}, Coverage Count: {coverage_count}, "
                      f"Time: {elapsed_time:.1f}s, Distance: {distance_to_wall:.2f}")

        rate.sleep()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

import rospy
from package_name.msg import Velocity
from std_msgs.msg import Float32

# Constants
MAX_SPEED = 60
TURN_SPEED = 50
WALL_THRESHOLD = 100
MIN_DISTANCE = 30
TURN_TIME = 2
MINI_FORWARD = 1.5

# Global variables
ultrasonic_read = float('inf')
last_distance = float('inf')
stable_readings_count = 0
zigzag_left = True
velocity_msg = Velocity()
current_state = 'MOVING_FORWARD'  # States: MOVING_FORWARD, TURNING_FIRST, MOVING_SHORT, TURNING_SECOND
coverage_count = 0
last_turn_time = 0
start_time = None

def range_callback(data):
    """Handle sensor readings with noise filtering"""
    global ultrasonic_read, last_distance, stable_readings_count
    
    # Filter sudden changes
    if abs(data.data - last_distance) > 50:
        stable_readings_count = 0
        return
        
    last_distance = data.data
    stable_readings_count += 1
    
    if stable_readings_count >= 3:  # Confirm reading is stable
        ultrasonic_read = data.data
        rospy.loginfo(f"Distance updated: {ultrasonic_read:.2f} cm")

def calculate_adaptive_speed():
    """Calculate speed with improved wall approach"""
    if ultrasonic_read > WALL_THRESHOLD:
        return MAX_SPEED
    elif MIN_DISTANCE <= ultrasonic_read <= WALL_THRESHOLD:
        # Smoother speed adjustment near walls
        normalized_distance = (ultrasonic_read - MIN_DISTANCE) / (WALL_THRESHOLD - MIN_DISTANCE)
        return MAX_SPEED * (0.4 + 0.6 * normalized_distance)
    return 0

def perform_turn_sequence(pub):
    """Execute improved turn sequence"""
    global current_state, zigzag_left, coverage_count, last_turn_time, velocity_msg
    
    current_time = rospy.get_time()
    
    if current_state == 'MOVING_FORWARD':
        # Prepare for first turn
        velocity_msg.x = 0
        velocity_msg.y = 0
        pub.publish(velocity_msg)
        rospy.sleep(0.5)  # Complete stop
        current_state = 'TURNING_FIRST'
        last_turn_time = current_time
        
    elif current_state == 'TURNING_FIRST':
        # First turn
        velocity_msg.x = 0 if zigzag_left else TURN_SPEED
        velocity_msg.y = TURN_SPEED if zigzag_left else 0
        
        if current_time - last_turn_time >= TURN_TIME:
            current_state = 'MOVING_SHORT'
            last_turn_time = current_time
            
    elif current_state == 'MOVING_SHORT':
        # Short forward movement
        velocity_msg.x = MAX_SPEED * 0.5
        velocity_msg.y = MAX_SPEED * 0.5
        
        if current_time - last_turn_time >= MINI_FORWARD:
            current_state = 'TURNING_SECOND'
            last_turn_time = current_time
            
    elif current_state == 'TURNING_SECOND':
        # Second turn
        velocity_msg.x = 0 if zigzag_left else TURN_SPEED
        velocity_msg.y = TURN_SPEED if zigzag_left else 0
        
        if current_time - last_turn_time >= TURN_TIME:
            current_state = 'MOVING_FORWARD'
            zigzag_left = not zigzag_left
            coverage_count += 1
            
    pub.publish(velocity_msg)

def control_node():
    global velocity_msg, current_state, start_time
    
    # Initialize ROS node
    rospy.init_node('coverage_controller', anonymous=True)
    pub = rospy.Publisher('velocity', Velocity, queue_size=10)
    rospy.Subscriber('Range', Float32, range_callback)
    
    rate = rospy.Rate(10)  
    start_time = rospy.get_time()
    
    while not rospy.is_shutdown():
        if current_state == 'MOVING_FORWARD':
            speed = calculate_adaptive_speed()
            velocity_msg.x = speed
            velocity_msg.y = speed
            
            if speed == 0:  # Wall detected
                perform_turn_sequence(pub)
            else:
                pub.publish(velocity_msg)
        else:
            perform_turn_sequence(pub)
            
        # print some info
        elapsed_time = rospy.get_time() - start_time
        rospy.loginfo(f"State: {current_state}, Coverage Count: {coverage_count}, "
                     f"Time: {elapsed_time:.1f}s, Distance: {ultrasonic_read:.1f}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass

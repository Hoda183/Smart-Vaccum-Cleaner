#!/usr/bin/env python3

import rospy
import os
from control_mode_pkg.srv import ControlMode, ControlModeResponse
from std_msgs.msg import Int32MultiArray


motors_speed_pub = rospy.Publisher('/motors_speed', Int32MultiArray, queue_size=10)
motors_speed_msg = Int32MultiArray()
motors_speed_msg.data = [0, 0] # breaks

def handle_control_mode(req):
    mode = req.mode
    rospy.loginfo(f"Received mode: {mode}")
    
    # nodes
    keyboard_node = "/keyboard_to_pwm" 
    ps3_node = "/ps3_motors" 
    autonomous_node = "/random_motion" 
    
    try:
        if mode == 1:
            # keyboard_node control
            os.system(f"rosnode kill {autonomous_node}")  
            os.system(f"rosnode kill {ps3_node}") 
            os.system(f"rosrun controller motors_test.py &")  # ????run keyboard control
            return ControlModeResponse(True, "Keyboard control activated.")
        
        elif mode == 2:
            # ps3_node control
            os.system(f"rosnode kill {keyboard_node}")  
            os.system(f"rosnode kill {autonomous_node}")  
            os.system(f"rosrun controller ps3_motors.py &")  
            return ControlModeResponse(True, "PS3 mode activated.")
        elif mode == 3:
            # autonomous_node control
            os.system(f"rosnode kill {keyboard_node}") 
            os.system(f"rosnode kill {ps3_node}")  
            os.system(f"rosrun controller random_motion.py &")  
            return ControlModeResponse(True, "Autonomous mode activated.")

        elif mode == 0:
            #  stop all nodes 
            os.system(f"rosnode kill {keyboard_node}")  #  stop keyboard control
            os.system(f"rosnode kill {ps3_node}")  
            os.system(f"rosnode kill {autonomous_node}")  #  stop autonomous control
            for i in range(5): # makes sure motors are stopped 
                motors_speed_pub.publish(motors_speed_msg)
            return ControlModeResponse(True, "All nodes stopped.")
        
        else:
            return ControlModeResponse(False, "Invalid mode. Use 0, 1, or 2.")
    
    except Exception as e:
        rospy.logerr(f"Error handling mode: {e}")
        return ControlModeResponse(False, f"Error: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('control_mode_server')
    service = rospy.Service('control_mode', ControlMode, handle_control_mode)
    rospy.loginfo("Control mode service is ready.")
    rospy.spin()

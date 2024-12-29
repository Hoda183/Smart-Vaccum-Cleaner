#!/usr/bin/env python3

import rospy
import os
from control_mode_pkg.srv import ControlMode, ControlModeResponse
from std_msgs.msg import Bool

motor_stop_pub = rospy.Publisher("stop_motor", Bool, queue_size=10)

def handle_control_mode(req):
    mode = req.mode
    rospy.loginfo(f"Received mode: {mode}")
    
    # nodes
    keyboard_node = "/keyboard_control" #??????????????
    autonomous_node = "/autonomous_control" #??????????????
    
    try:
        if mode == 1:
            # keyboard_node control
            os.system(f"rosnode kill {autonomous_node}")  # stop autonomous control
            os.system(f"rosrun your_package_name keyboard_control.py &")  # ????run keyboard control
            motor_stop_pub.publish(True)
            return ControlModeResponse(True, "Keyboard control activated.")
        
        elif mode == 2:
            # autonomous_node control
            os.system(f"rosnode kill {keyboard_node}")  # stop keyboard control
            os.system(f"rosrun your_package_name autonomous_control.py &")  # ????run autonomous control
            motor_stop_pub.publish(False)
            return ControlModeResponse(True, "Autonomous mode activated.")
        
        elif mode == 0:
            #  stop all nodes 
            os.system(f"rosnode kill {keyboard_node}")  #  stop keyboard control
            os.system(f"rosnode kill {autonomous_node}")  #  stop autonomous control
            motor_stop_pub.publish(False)
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

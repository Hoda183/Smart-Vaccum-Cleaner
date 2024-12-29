#!/usr/bin/env python3

import rospy
from control_mode_pkg.srv import ControlMode

def set_mode(mode):
    rospy.wait_for_service('control_mode')
    try:
        control_service = rospy.ServiceProxy('control_mode', ControlMode)
        response = control_service(mode)
        if response.success:
            rospy.loginfo(response.message)
        else:
            rospy.logwarn(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('control_mode_client')
    mode = int(input("Enter mode (0: Stop, 1: Keyboard, 2: Autonomous): "))
    set_mode(mode)
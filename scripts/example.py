#!/usr/bin/env python3
from controler import Flight_controller
import time
import rospy
if __name__ == '__main__':
    ic = Flight_controller()
    rate = rospy.Rate(10)
    ic.toggle_arm(True)
    #ic.set_mode('STABILIZE')
    ic.set_offboard_mode()
    # time.sleep(2)
    # ic.takeoff(3)
    ic.move_to(10,1,3)

    x= 1
    for i in range (0,1000):
        ic.accel_command(x,0,0)
        rate.sleep()

#!/usr/bin/env python

"""
Script publish the same cmd to multiple robots
"""

import numpy as np
import rospy
import rosnode
from std_msgs.msg import Int8, Bool
from ar_commander.msg import ControllerCmd


RATE = 70

Omega = np.ones(4)*0.4
Phi = np.ones(4)*0#.15

rospy.init_node('cmd_publisher')

pub_cmds_1 = rospy.Publisher('/robot1/controller_cmds', ControllerCmd, queue_size=10)
pub_cmds_2 = rospy.Publisher('/robot2/controller_cmds', ControllerCmd, queue_size=10)
pub_cmds_3 = rospy.Publisher('/robot3/controller_cmds', ControllerCmd, queue_size=10)
pub_cmds_4 = rospy.Publisher('/robot4/controller_cmds', ControllerCmd, queue_size=10)

def publish():
        cmd1 = ControllerCmd()
        cmd2 = ControllerCmd()
        cmd3 = ControllerCmd()
        cmd4 = ControllerCmd()

        cmd1.omega_arr.data = Omega
        cmd1.phi_arr.data = Phi + np.pi/2# - 2*Phi
        cmd2.omega_arr.data = Omega
        cmd2.phi_arr.data = Phi
        cmd3.omega_arr.data = Omega
        cmd3.phi_arr.data = Phi
        cmd4.omega_arr.data = Omega
        cmd4.phi_arr.data = Phi #+ np.pi/2

        pub_cmds_1.publish(cmd1)
        pub_cmds_2.publish(cmd2)
        pub_cmds_3.publish(cmd3)
        pub_cmds_4.publish(cmd4)


def run():
    rate = rospy.Rate(RATE) # 10 Hz
    while not rospy.is_shutdown():
        publish()
        rate.sleep()


if __name__ == '__main__':
    run()

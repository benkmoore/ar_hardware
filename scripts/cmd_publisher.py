#!/usr/bin/env python

"""
Script publish the same cmd to multiple robots
"""

import numpy as np
import time
import rospy
import rosnode

from std_msgs.msg import Int8, Bool
from ar_commander.msg import ControllerCmd


RATE = 70
OMEGA = 0.4


class Commander():

    def __init__(self):
        rospy.init_node('cmd_publisher')

        self.pub_cmds_1 = rospy.Publisher('/robot1/controller_cmds', ControllerCmd, queue_size=10)
        self.pub_cmds_2 = rospy.Publisher('/robot2/controller_cmds', ControllerCmd, queue_size=10)
        self.pub_cmds_3 = rospy.Publisher('/robot3/controller_cmds', ControllerCmd, queue_size=10)
        self.pub_cmds_4 = rospy.Publisher('/robot4/controller_cmds', ControllerCmd, queue_size=10)

        self.cmd1 = ControllerCmd()
        self.cmd2 = ControllerCmd()
        self.cmd3 = ControllerCmd()
        self.cmd4 = ControllerCmd()

        self.start = time.time()

        self.Omega = np.ones(4)*OMEGA
        self.Phi = np.ones(4)*0#.15

        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.Phi #+ np.pi/2# - 2*Phi
        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.Phi
        self.cmd3.omega_arr.data = self.Omega
        self.cmd3.phi_arr.data = self.Phi
        self.cmd4.omega_arr.data = self.Omega
        self.cmd4.phi_arr.data = self.Phi


    def publish(self):
        self.pub_cmds_1.publish(self.cmd1)
        self.pub_cmds_2.publish(self.cmd2)
        self.pub_cmds_3.publish(self.cmd3)
        self.pub_cmds_4.publish(self.cmd4)

    def spin(self):
        self.Omega = np.ones(4)*OMEGA
        self.Phi = np.ones(4)*(-np.pi/6)#.15

        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.Phi #+ np.pi/2# - 2*Phi
        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.Phi
        self.cmd3.omega_arr.data = self.Omega
        self.cmd3.phi_arr.data = self.Phi
        self.cmd4.omega_arr.data = self.Omega
        self.cmd4.phi_arr.data = self.Phi #+ np.pi/2

    def straight(self):
        self.Phi = np.ones(4)*0#.15
        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.Phi + np.pi/2# - 2*Phi
        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.Phi 
        self.cmd3.omega_arr.data = self.Omega
        self.cmd3.phi_arr.data = self.Phi + np.pi
        self.cmd4.omega_arr.data = self.Omega
        self.cmd4.phi_arr.data = self.Phi 

    def reverse(self):
        self.straight()
        self.Phi -= np.pi

    def U(self):
        self.straight()
        if(time.time() - self.start) > 4:
            self.reverse()
        elif(time.time() - self.start) > 2:
            self.Omega -= np.pi/2
        

    def run(self):
        rate = rospy.Rate(RATE) 
        while not rospy.is_shutdown():
            self.U()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    commander = Commander()
    commander.run()

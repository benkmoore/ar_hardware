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
OMEGA = 0.32
OFFSET = np.pi/4 #angle required to make robot in leftmost/ bottom left position align with x & y axis (0 degrees)


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

        self.Omega = np.ones(4) * OMEGA
        self.phi = np.ones(4) * 0#.15

        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.phi #+ np.pi/2# - 2*phi
        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.phi
        self.cmd3.omega_arr.data = self.Omega
        self.cmd3.phi_arr.data = self.phi
        self.cmd4.omega_arr.data = self.Omega 
        self.cmd4.phi_arr.data = self.phi


    def publish(self):
        self.pub_cmds_1.publish(self.cmd1)
        self.pub_cmds_2.publish(self.cmd2)
        self.pub_cmds_3.publish(self.cmd3)
        self.pub_cmds_4.publish(self.cmd4)

    def spin(self):
        self.Omega = np.ones(4) * OMEGA
        self.phi = np.ones(4) * (-np.pi/6)#.15

        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.phi #+ np.pi/2# - 2*phi
        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.phi
        self.cmd3.omega_arr.data = self.Omega
        self.cmd3.phi_arr.data = self.phi
        self.cmd4.omega_arr.data = self.Omega
        self.cmd4.phi_arr.data = self.phi #+ np.pi/2

    def move(self, angle):
        self.phi = np.ones(4) * 0#.15
        self.cmd1.omega_arr.data = self.Omega
        self.cmd1.phi_arr.data = self.phi + np.pi/2# - 2*phi

        self.cmd2.omega_arr.data = self.Omega
        self.cmd2.phi_arr.data = self.phi 

        #currently robot4 is in left pos (pos 1), r3 in right pos 
        self.cmd4.omega_arr.data = self.Omega
        self.cmd4.phi_arr.data = self.phi + angle

        self.cmd3.omega_arr.data = self.Omega 
        self.cmd3.phi_arr.data = self.cmd4.phi_arr.data + np.pi
        

    def up(self):
        self.move(OFFSET + np.pi/2)

    def down(self):
        self.move(OFFSET - np.pi/2)

    def right(self):
        self.move(OFFSET)
        
    def left(self):
        self.move(OFFSET - np.pi*1.1)

    def square(self):
        if(time.time() - self.start) > 16:
            self.Omega *= 0
        elif(time.time() - self.start) > 14:
            self.left()
        elif(time.time() - self.start) > 10:
            self.down()
        elif(time.time() - self.start) > 6:
            self.right()
            print("turning")
        else:
            self.up()
            print("move")        

    def U(self):
        if(time.time() - self.start) > 8:
            self.down()
            print("reversing")
        elif(time.time() - self.start) > 4:
            self.right()
            print("turning")
        else:
            self.up()
            print("move")

    def longU(self):
        if(time.time() - self.start) > 25:
            self.Omega *= 0
        elif(time.time() - self.start) > 18.5:
            self.up()
        elif(time.time() - self.start) > 14:
            self.right()
        elif(time.time() - self.start) > 10:
            self.up()
        elif(time.time() - self.start) > 6:
            self.left()
            print("turning")
        else:
            self.up()
            print("move")


    def run(self):
        rate = rospy.Rate(RATE) 
        while not rospy.is_shutdown():
            self.up()
            # self.longU()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    commander = Commander()
    commander.run()

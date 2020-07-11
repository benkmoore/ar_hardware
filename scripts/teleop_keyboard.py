#!/usr/bin/env python

from __future__ import print_function

import threading
import numpy as np
import numpy.linalg as npl
import roslib
import rospy

from ar_commander.msg import ControllerCmd

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to controller_cmds!
---------------------------

Keys -> direction of movement

q w e
a s d
z x c

Stop: s

CTRL-C to stop motors and quit
"""

N = 4 				# number of drive motors
R1 = np.array([0.075, 0.425])   # position of wheels along arm 1
R2 = np.array([0.075, 0.425])   # "" arm 2


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.pub_cmds = rospy.Publisher('/controller_cmds', ControllerCmd, queue_size=10)
	self.cmds = ControllerCmd()
        self.V_cmd = np.zeros(N);
        self.phi_cmd = np.zeros(N);
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.pub_cmds.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.pub_cmds.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, V_cmd, phi_cmd):
        self.condition.acquire()
        self.cmds.velocity_arr.data = V_cmd
        self.cmds.phi_arr.data = phi_cmd
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(np.zeros(N), np.zeros(N))
        self.join()

    def run(self):
        cmds = ControllerCmd()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
	    self.condition.release()
	    self.pub_cmds.publish(self.cmds)

        # Publish stop message when thread exits
        cmds.velocity_arr.data = np.zeros(N);
        cmds.phi_arr.data = np.zeros(N);
        self.pub_cmds.publish(self.cmds)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def cmds(V_cmd, phi_cmd):
    return "currently:\tV_cmd %s\tphi_cmd %s " % (V_cmd, phi_cmd)

def convert2MotorInputs(v_cmd_gf, omega_cmd):
    """Convert velocity and omega commands to motor inputs"""
    
    # convert inputs to robot frame velocity commands
    R = np.array([[np.cos(0), np.sin(0)],     # rotation matrix
                 [-np.sin(0), np.cos(0)]])
    v_cmd_rf = np.dot(R, v_cmd_gf)[:,np.newaxis]        # convert to robot frame
    v_th1 = np.vstack([-R1*omega_cmd, np.zeros(N/2)])
    v_th2 = np.vstack([np.zeros(N/2), R2*omega_cmd])
    v_th_rf = np.hstack([v_th1, v_th2])

    V_cmd = v_cmd_rf + v_th_rf

    # Convert to |V| and phi
    V_norm_cmd = npl.norm(V_cmd, axis=0)
    phi_cmd = np.arctan2(V_cmd[1,:], V_cmd[0,:]) + np.pi/2

    return V_norm_cmd, phi_cmd

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    repeat = rospy.get_param("~repeat_rate", 50.0)
    print("Repeat rate cmds: %d Hz" % repeat)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
	
    vel = 170;	

    V_cmd = np.zeros(N);
    phi_cmd = np.zeros(N);

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(V_cmd, phi_cmd)

        print(msg)
	print(cmds(V_cmd, phi_cmd))
        while(1):
            key = getKey(key_timeout)
            if key == 'q':
		print("Forward left...")
		v_cmd = np.array([vel,vel])
		omega_cmd = 0
            elif key == 'w':
		print("Forward...")
		v_cmd = np.array([0,-vel])
		omega_cmd = 0
            elif key == 'e':
		print("Forward right...")
		v_cmd = np.array([-vel,vel])
		omega_cmd = 0
            elif key == 'a':
		print("Left...")
		v_cmd = np.array([vel,0])
		omega_cmd = 0
            elif key == 'd':
		print("Right...")
		v_cmd = np.array([-vel,0])
		omega_cmd = 0
            elif key == 'x':
		print("Backward...")
		v_cmd = np.array([0,vel])
		omega_cmd = 0
            elif key == 'c':
		print("Backward right...")
		v_cmd = np.array([vel,-vel])
		omega_cmd = 0
            elif key == 'z':
		print("Backward left...")
		v_cmd = np.array([-vel,-vel])
            elif key == 's':
		print("Stopping...")
		v_cmd = np.array([0,0])
		omega_cmd = 0
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and V_cmd == np.zeros(N) and phi_cmd == np.zeros(N):
                    continue
                if (key == '\x03'):
		    print("Stopping and exiting...")
                    break
 	    V_cmd, phi_cmd = convert2MotorInputs(v_cmd, omega_cmd)
            pub_thread.update(V_cmd, phi_cmd)
            print(cmds(V_cmd, phi_cmd))

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

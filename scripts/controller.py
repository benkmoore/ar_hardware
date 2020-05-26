#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, Vector3
# from tf.transformations import euler_from_quaternion

# Global variables:
h = [0.35, 0.125, 0.35, 0.125]                  # wheel distances along arms (from end) [ L1 L2 R1 R2 ]
N = len(h)                                      # number drive motors
d = np.sqrt(2*(0.475**2))                       # diagonal distance between two arms of robot


def pointController(self, p_des, p_cur, v_cur, theta_des, theta_cur):
    kp_1 = 10
    kd_1 = 0
    kp_2 = 3
    ki_2 = 0.03
    kd_2 = 5*(10**-8)
    p_delta = p_des-p_cur
    theta_delta = theta_des-theta_cur
    self.theta_error_sum += theta_delta

    if npl.norm(p_delta) > 0.01:
        v_cmd = kp_1*p_delta + kd_1*(p_delta-self.p_delta_prev)
    else:
        v_cmd = np.array([10**-15,10**-15])
    self.theta_dot_cmd = kp_2*theta_delta + ki_2*self.theta_error_sum
    + kd_2*(theta_delta-self.theta_delta_prev)

    self.theta_delta_prev = theta_delta
    self.p_delta_prev = p_delta

    # Convert to motor inputs
    phi_cmd = np.arctan2(v_cmd[1], v_cmd[0]) + np.pi/2

    l = d*np.sin((np.pi/2)+phi_cmd)
    delta_v = self.theta_dot_cmd*l

    v_1 =  npl.norm(v_cmd)-delta_v/2
    v_end =   npl.norm(v_cmd)+delta_v/2
    V_cmd = np.array([v_1,v_1,v_end,v_end]) # [ L1 L2 R1 R2 ]

    print("v_cmd: ",v_cmd,", theta_dot_cmd: ", self.theta_dot_cmd," delta_v: ", delta_v)
    print("V: ", V_cmd, "phi_cmd: ", phi_cmd)
    return V_cmd, phi_cmd


class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.pos = None
        self.q = None # quaternion
        self.theta = None
        self.vel = None
        self.motor_pos_arr = None
        self.motor_vel_arr = None
        self.motor_names = None
        self.phi_cmd = None
        self.V_cmd = None
        self.theta_dot_cmd = 0
        self.theta_error_sum = 0
        self.theta_delta_prev = 0
        self.p_delta_prev = 0

        self.pos_des = Vector3()
        self.pos_des.x = 0
        self.pos_des.y = 6
        self.pos_des.z = 0
        self.theta_des = 1

        # publishers
        self.cmdP1_pub = rospy.Publisher("/robot_0/joint1_position_controller/command",Float64, queue_size=10)
        self.cmdP2_pub = rospy.Publisher("/robot_0/joint2_position_controller/command",Float64, queue_size=10)
        self.cmdP3_pub = rospy.Publisher("/robot_0/joint3_position_controller/command",Float64, queue_size=10)
        self.cmdP4_pub = rospy.Publisher("/robot_0/joint4_position_controller/command",Float64, queue_size=10)
        self.cmdV5_pub = rospy.Publisher("/robot_0/joint5_velocity_controller/command",Float64, queue_size=10)
        self.cmdV6_pub = rospy.Publisher("/robot_0/joint6_velocity_controller/command",Float64, queue_size=10)
        self.cmdV7_pub = rospy.Publisher("/robot_0/joint7_velocity_controller/command",Float64, queue_size=10)
        self.cmdV8_pub = rospy.Publisher("/robot_0/joint8_velocity_controller/command",Float64, queue_size=10)

        # subscribers
        rospy.Subscriber('/robot_0/joint_states', JointState, self.joint_statesCallback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_statesCallback)

    def model_statesCallback(self, msg):
        self.pos = msg.pose[1].position
        self.q = msg.pose[1].orientation
        # _, _, self.theta = euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])
        self.vel = msg.twist[1].linear
        self.ang_vel = msg.twist[1].angular

    def joint_statesCallback(self, msg):
        self.motor_names = msg.name
        self.motor_pos_arr = msg.position
        self.motor_vel_arr = msg.velocity

    def controlLoop(self):
        # default behavior (0 = Vx Vy theta_dot)
        self.V_cmd = np.zeros(N)
        self.phi_cmd = 0 # rads

        if self.pos != None:
            p_des = np.array([self.pos_des.x,self.pos_des.y])
            p_cur = np.array([self.pos.x,self.pos.y])
            v_cur = np.array([self.vel.x,self.vel.y])

            self.V_cmd, self.phi_cmd = pointController(self, p_des, p_cur, v_cur, self.theta_des, self.theta)
            print("p_d: ", p_des,", p_cur: ", p_cur,", theta: ",self.theta, " theta_d: ", self.theta_des)

    def publish(self):
        """ publish cmd messages """
        self.cmdP1_pub.publish(self.phi_cmd)    # yaw_joint_l1
        self.cmdP2_pub.publish(self.phi_cmd)    # yaw_joint_l2
        self.cmdP3_pub.publish(self.phi_cmd)    # yaw_joint_r1
        self.cmdP4_pub.publish(self.phi_cmd)    # yaw_joint_r2
        self.cmdV5_pub.publish(self.V_cmd[0])   # drive_joint_l1
        self.cmdV6_pub.publish(self.V_cmd[1])   # drive_joint_l2
        self.cmdV7_pub.publish(self.V_cmd[2])   # drive_joint_r1
        self.cmdV8_pub.publish(self.V_cmd[3])   # drive_joint_r2

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.run()

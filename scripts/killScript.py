#!/usr/bin/env python

"""
Script to stop robots listening to controller cmds and stop moving
"""

import rospy
import rosnode
from std_msgs.msg import Int8, Bool

RATE = 70


rospy.init_node('kill_script')

pub_kill = rospy.Publisher('kill_script', Int8, queue_size=10)

def publish():
        msg = Int8()
        msg.data = 1
        pub_kill.publish(msg)


def run():
    rate = rospy.Rate(RATE) # 10 Hz
    while not rospy.is_shutdown():
        publish()
        rate.sleep()


if __name__ == '__main__':
    run()

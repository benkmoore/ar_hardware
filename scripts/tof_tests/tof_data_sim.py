#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from ar_commander.msg import TOF, State

tof = TOF()
state = State()
def tofSensors():
    rospy.init_node('tof_data', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pubTOF = rospy.Publisher('tof_data', TOF, queue_size=10)
    pubState = rospy.Publisher('estimator/state', State, queue_size=10)
    print 'started'

    while not rospy.is_shutdown():
        # tof = np.random.randint(1,100, size=(1,3))

        state.pos.data = np.zeros(2)
        state.theta.data = 0#np.pi/4
        state.vel.data = np.zeros(2)
        state.omega.data = 0

        #tof.tof1 = np.random.randint(50,200)
        tof.tof2 = 1000#np.random.randint(250,400)
        #tof.tof3 = np.random.randint(50,200)
        rospy.loginfo_throttle(3, tof)


        pubTOF.publish(tof)
        pubState.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        tofSensors()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python


import rospy
import numpy as np
from ar_commander.msg import TOF, Trajectory, State
import matplotlib.pyplot as plt
from std_msgs.msg import String


#maybe we should have an instance for each tof sensor rather than doing them all in one.
class SensorNode():
    
    def __init__(self):
        rospy.init_node('XY_data', anonymous=True)

        self.posLeft = [0,400]
        self.posMid = [0,0]
        self.posRight = [400,0]
        self.angleL = 0
        self.angleM = 45
        self.angleR = 90
        self.xyLeft = [0,0]
        self.xyMid = [0,0]
        self.xyRight = [0,0]
        # print self.xyLeft, self.xyMid, self.xyRight



    def xyCallback(self, tof_data):

        sinMid = np.sin(np.deg2rad(self.angleM))
        cosMid = np.cos(np.deg2rad(self.angleM))
        sinLeft = np.sin(np.deg2rad(self.angleL))
        cosLeft = np.cos(np.deg2rad(self.angleL))
        sinRight = np.sin(np.deg2rad(self.angleR))
        cosRight = np.cos(np.deg2rad(self.angleR))

            #left refers to the sensor on the y axis, right is the sensor on the x 
            #when robot corner is at 0,0 left arm goes from (0,0) to (0,450) in mm

        left_tof = tof_data.tof1
        mid_tof = tof_data.tof2
        right_tof = tof_data.tof3


        self.xyLeft = [round(left_tof*cosLeft + self.posLeft[0],2), round(left_tof*sinLeft + self.posLeft[1],2)]
        self.xyMid = [round(mid_tof*sinMid + self.posMid[0],2), round(mid_tof*cosMid + self.posMid[1],2)]
        self.xyRight = [round(right_tof*cosRight + self.posRight[0],2), round(right_tof*sinRight + self.posRight[1],2)]
        # print self.xyLeft, self.xyMid, self.xyRight

    def main(self):
        rospy.Subscriber('tof_data', TOF, self.xyCallback)
        self.rate = float(rospy.get_param('~rate', 10.0))
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown(): #and self.xyLeft != None:
                plt.scatter(self.xyLeft[0],self.xyLeft[1])
                plt.scatter(self.xyMid[0],self.xyMid[1])        
                plt.scatter(self.xyRight[0],self.xyRight[1])

                # i+=1;
                plt.show()
                plt.pause(0.0001)
                rate.sleep()

                print self.xyLeft, self.xyMid, self.xyRight
        rospy.spin()




if __name__ == '__main__':
    try:
        plt.ion()
        fig=plt.figure()
        plt.axis([0,400,0,400])

        sensors = SensorNode()
        sensors.main()
    except rospy.ROSInterruptException:
        pass
 
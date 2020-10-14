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
        self.posLeft = None
        self.posMid = None
        self.posRight = None
        self.angleL = 0
        self.angleM = np.pi/4
        self.angleR = np.pi/2
        self.xyLeft = None
        self.xyMid = None
        self.xyRight = None
        self.theta = 0
        # print self.xyLeft, self.xyMid, self.xyRight

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

        sinM = np.sin((self.theta+np.pi/4))
        cosM = np.cos((self.theta+np.pi/4))
        sinL = np.sin((self.theta+1.249))
        cosL = np.cos((self.theta+1.249))
        sinR = np.sin((self.theta+0.322))
        cosR = np.cos((self.theta+0.322))

        self.absLeft = np.sqrt(150**2+450**2)
        self.absMid = np.sqrt(150**2+150**2)
        self.absRight = np.sqrt(450**2+150**2)

        self.posLeft = self.pos*1000 + [cosL*self.absLeft,sinL*self.absLeft]
        self.posMid = self.pos*1000 + [cosM*self.absMid,sinM*self.absMid]
        self.posRight = self.pos*1000 + [cosR*self.absRight,sinR*self.absRight]


    def xyCallback(self, tof_data):
        sinMid = np.sin((self.angleM + self.theta))
        cosMid = np.cos((self.angleM + self.theta))
        sinLeft = np.sin((self.angleL + self.theta))
        cosLeft = np.cos((self.angleL + self.theta))
        sinRight = np.sin((self.angleR + self.theta))
        cosRight = np.cos((self.angleR + self.theta))

            #left refers to the sensor on the y axis, right is the sensor on the x 
            #when robot corner is at 0,0 left arm goes from (0,0) to (0,450) in mm

        left_tof = tof_data.tof1
        mid_tof = tof_data.tof2
        right_tof = tof_data.tof3

        if self.posLeft is not None:
            self.xyLeft = [round(left_tof*cosLeft + self.posLeft[0],2), round(left_tof*sinLeft + self.posLeft[1],2)]
            self.xyMid = [round(mid_tof*cosMid + self.posMid[0],2), round(mid_tof*sinMid + self.posMid[1],2)]
            self.xyRight = [round(right_tof*cosRight + self.posRight[0],2), round(right_tof*sinRight + self.posRight[1],2)]

    def main(self):
        rospy.Subscriber('tof_data', TOF, self.xyCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        self.rate = float(rospy.get_param('~rate', 70.0))
        rate = rospy.Rate(self.rate)
      
        while not rospy.is_shutdown(): #and self.xyLeft != None:
            if self.xyLeft is not None:
                plt.scatter(self.xyLeft[0],self.xyLeft[1],marker = ".")
                plt.scatter(self.xyMid[0],self.xyMid[1],marker = ".")        
                plt.scatter(self.xyRight[0],self.xyRight[1],marker = ".")

                plt.scatter(self.posLeft[0], self.posLeft[1], c = "g", marker = "x")
                plt.scatter(self.posMid[0], self.posMid[1], c = "g", marker = "x")        
                plt.scatter(self.posRight[0], self.posRight[1], c = "g", marker = "x")

                plt.xlabel("x value in mm")
                plt.ylabel("y value in mm")
                plt.show()
                plt.pause(0.0001)
            rate.sleep()

#                print self.posMid, self.xyMid, self.theta
        rospy.spin()




if __name__ == '__main__':
    try:
        plt.ion()
       
        fig=plt.figure()
        plt.axis([-1000,1000,-1000,1000])

        sensors = SensorNode()
        sensors.main()
    except rospy.ROSInterruptException:
        pass
 

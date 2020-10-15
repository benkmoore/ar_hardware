#!/usr/bin/env python


import rospy
import numpy as np
from ar_commander.msg import TOF, Trajectory, State
import matplotlib.pyplot as plt
from std_msgs.msg import String

class DepthSensor():
    
    def __init__(self, idNo, relativPos, relativAngle):
        rospy.init_node('XY_data', anonymous=True)

        self.absolutPos = None
        self.relativPos = relativPos
        self.relativAngle = relativAngle        
        self.xyData = None
        self.theta = None
        self.idNo = idNo
        self.sensorData = None
        # print self.xyLeft, self.xyMid, self.xyRight

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

        sinVal = np.sin((self.theta+np.arctan(self.relativPos[1]/self.relativPos[0]))) #0.322 for xarm sensor, 1.249 for y arm sensor 0.784 for mid sensor
        cosVal = np.cos((self.theta+np.arctan(self.relativPos[1]/self.relativPos[0]))) #0.322 for xarm sensor, 1.249 for y arm sensor 0.784 for mid sensor
        self.absDist = np.sqrt(self.relativPos[0]**2 + self.relativPos[1]**2)# np.linalg      
        self.absolutPos = self.pos*1000 + [cosVal*self.absDist,sinVal*self.absDist]
       

    def xyCallback(self, tof_data):  
        if self.idNo == 1:
            self.sensorData = tof_data.tof1
        elif self.idNo == 2:
            self.sensorData = tof_data.tof2
        elif self.idNo == 3:
            self.sensorData = tof_data.tof3

       

        if self.absolutPos is not None and self.sensorData is not None:
            self.xyData = [round(self.sensorData*np.cos((self.relativAngle + self.theta)) + self.absolutPos[0],2), round(self.sensorData*np.sin((self.relativAngle + self.theta)) + self.absolutPos[1],2)]
           
    def main(self):
        rospy.Subscriber('tof_data', TOF, self.xyCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        self.rate = float(rospy.get_param('~rate', 70.0))
        rate = rospy.Rate(self.rate)
      
        while not rospy.is_shutdown(): #and self.xyLeft != None:
            if self.xyData is not None:
                plt.scatter(self.xyData[0],self.xyData[1],marker = ".")
                
                plt.scatter(self.absolutPos[0], self.absolutPos[1], c = "g", marker = "x")
                
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

        sensor1 = DepthSensor(1, [150,450], 0)
        # sensor2 = DepthSensor(2, [150,150], np.pi/4)
        # sensor3 = DepthSensor(3, [450,150], np.pi/2)

        sensor1.main()
    except rospy.ROSInterruptException:
        pass
 

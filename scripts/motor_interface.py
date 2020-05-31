#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

from stepperMotor import *

# lists of dc and stepper motor pins
dc_pins = [[]] #[[1,2,3],[4,5,6]]					# 3 pins [in1, in2, enable]
stepper_pins = [[]] #[[5,6,7,8],[9,10,11,12]]		# 5 pins [A1, A2, B1, B2, enable]
OP_LIMIT = 90 								# Operation limit of motor, 0 - 100 %

class MotorInterface():
	def __init__(self):
		rospy.init_node('motor_interface')
		rospy.Subscriber('/controller_cmds', ControllerCmd, self.control_cmdsCallback)

		self.V_cmd = None
		self.phi_cmd = None
		self.phi_curr = 0

		# Setup GPIO pins
		GPIO.setmode(GPIO.BOARD) # Use pin numbers: https://circuitdigest.com/microcontroller-projects/controlling-stepper-motor-with-raspberry-pi
		v_cmders = [] # list of vel commanders for each motor
		for i, motor_pins in enumerate(dc_pins, start=0):
			# Setup
			GPIO.setup(motor_pins[0], GPIO.OUT)
			GPIO.setup(motor_pins[1], GPIO.OUT)
			GPIO.setup(motor_pins[2], GPIO.OUT)
			v_cmders.append(GPIO.PWM(motor_pins[2],1000))
			# Init output - 0
			GPIO.output(motor_pins[0],GPIO.LOW)
			GPIO.output(motor_pins[1],GPIO.LOW)
			v_cmders[i].start(0)

		for motor_pins in stepper_pins:
			GPIO.setup(motor_pins[0], GPIO.OUT)
			GPIO.setup(motor_pins[1], GPIO.OUT)
			GPIO.setup(motor_pins[2], GPIO.OUT)
			GPIO.setup(motor_pins[3], GPIO.OUT)


	def control_cmdsCallback(self, msg):
		self.V_cmd = msg.velocity_arr.data
		self.phi_cmd = msg.phi_arr.data[0] #[phi1, phi2]

	def cmdMotors(self):
		# do we want to use a mutex lock here so we get the cmds out without delay?
		for i, motor_pins in enumerate(dc_pins, start=0):
			self.cmdDCMotor(motor_pins, i)

		for motor_pins in stepper_pins:
			self.cmdStepperMotor(motor_pins)

	def cmdStepperMotor(self, motor_pins):
		if self.phi_cmd != None:
			delta_phi = self.phi_cmd-self.phi_curr
			if abs(delta_phi) < 10**-3:
				pass
			elif delta_phi > 0:
				for i in range(0,len(stepper_pins)):
					forward(delay, stepper_pins[i], delta_phi)
			elif delta_phi < 0:
				for i in range(0,len(stepper_pins)):
					backward(delay, stepper_pins[i], delta_phi)

			# TODO: Need to encode current phi to tell steppers to turn desired amount
			self.phi_cur = self.phi_cmd


	def cmdDCMotor(self, motor_pins, i):
		if self.V_cmd != None:
			if abs(self.V_cmd[i]) < 10**-2: # deadband
				GPIO.output(motor_pins[0],GPIO.LOW)
				GPIO.output(motor_pins[1],GPIO.LOW)
			elif self.V_cmd[i] > 0: # pos vels -> fwd
				GPIO.output(motor_pins[0],GPIO.HIGH)
				GPIO.output(motor_pins[1],GPIO.LOW)
				v_cmders[i].ChangeDutyCycle(max(OP_LIMIT,abs(self.V_cmd[i])))
			elif self.V_cmd[i] < 0: # neg vels -> bwd
				GPIO.output(in1,GPIO.LOW)
				GPIO.output(in2,GPIO.HIGH)
				v_cmders[i].ChangeDutyCycle(max(OP_LIMIT,abs(self.V_cmd[i])))

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		try:
			while not rospy.is_shutdown():
				interface.cmdMotors()
				rate.sleep()
		except:
			for i in range(0,len(v_cmders)):
				v_cmders[i].stop()
			GPIO.cleanup()

if __name__ == '__main__':
	interface = MotorInterface()
	interface.run()

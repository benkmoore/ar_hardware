#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
import signal
import time

from ar_commander.msg import ControllerCmd

# lists of dc and stepper motor pins
dc_pins = [[35,37,33]]			# 3 pins [in1, in2, enable]
stepper_pins = [[23,21,26,19]]		# 4 pins [A1, A2, B1, B2]
OP_LIMIT = 90 				# Operation limit of motor, 0 - 100 %
STEP_DELAY = 0.001			# Delay between steps
PHI_STEP = 360/405				# Resolution of stepper motor, 360 degrees = 405 steps

StepCount = 8				# Encode motor step sequence
Seq = range(0, StepCount)
Seq[0] = [1,0,0,0]
Seq[1] = [1,1,0,0]
Seq[2] = [0,1,0,0]
Seq[3] = [0,1,1,0]
Seq[4] = [0,0,1,0]
Seq[5] = [0,0,1,1]
Seq[6] = [0,0,0,1]
Seq[7] = [1,0,0,1]

class MotorInterface():
	def __init__(self):
		rospy.init_node('motor_interface')
		rospy.Subscriber('/controller_cmds', ControllerCmd, self.control_cmdsCallback)

		self.V_cmd = None
		self.phi_cmd = None
		self.phi_cur = 0

		# Setup GPIO pins
		GPIO.setmode(GPIO.BOARD) # Use pin numbers: https://circuitdigest.com/microcontroller-projects/controlling-stepper-motor-with-raspberry-pi
		self.v_cmders = [] # list of vel commanders for each motor
		for i, motor_pins in enumerate(dc_pins, start=0):
			# Setup
			GPIO.setup(motor_pins[0], GPIO.OUT)
			GPIO.setup(motor_pins[1], GPIO.OUT)
			GPIO.setup(motor_pins[2], GPIO.OUT)
			self.v_cmders.append(GPIO.PWM(motor_pins[2],1000))
			# Init output - 0
			GPIO.output(motor_pins[0],GPIO.LOW)
			GPIO.output(motor_pins[1],GPIO.LOW)
			self.v_cmders[i].start(0)

		for motor_pins in stepper_pins:
			GPIO.setup(motor_pins[0], GPIO.OUT)
			GPIO.setup(motor_pins[1], GPIO.OUT)
			GPIO.setup(motor_pins[2], GPIO.OUT)
			GPIO.setup(motor_pins[3], GPIO.OUT)


	def control_cmdsCallback(self, msg):
		self.V_cmd = msg.velocity_arr.data
		self.phi_cmd = msg.phi_arr.data[0] #[phi1, phi2]

	def step(self, pins, cmd):
		GPIO.output(pins[0], cmd[0])
		GPIO.output(pins[1], cmd[1])
		GPIO.output(pins[2], cmd[2])
		GPIO.output(pins[3], cmd[3])

	def step_forward(self, delay, steps, pins):
		for i in range(steps):
			for j in range(StepCount):
				self.step(pins, Seq[j][:])
				time.sleep(delay)

	def step_backward(self, delay, steps, pins):
		for i in range(steps):
			for j in reversed(range(StepCount)):
				self.step(pins, Seq[j][:])
				time.sleep(delay)

	def cmdMotors(self):
		for i, motor_pins in enumerate(dc_pins, start=0):
			self.cmdDCMotor(motor_pins, i)

		for motor_pins in stepper_pins:
			self.cmdStepperMotor(motor_pins)

	def cmdStepperMotor(self, motor_pins):
		if self.phi_cmd != None:
			delta_phi = self.phi_cmd-self.phi_cur
			num_steps = int(delta_phi/PHI_2_STEP)# must be int num of steps
			if num_steps == 0:
				pass
			elif num_steps > 0:
				for i in range(0,len(stepper_pins)):
					self.step_forward(STEP_DELAY, num_steps, stepper_pins[i])
			elif num_steps < 0:
				for i in range(0,len(stepper_pins)):
					self.step_backward(STEP_DELAY, abs(num_steps), stepper_pins[i])

			# TODO: Need to encode current phi to tell steppers to turn desired amount
			self.phi_cur = self.phi_cur + delta_phi


	def cmdDCMotor(self, motor_pins, i):
		if self.V_cmd != None:
			if abs(self.V_cmd[i]) < 10**-2: # deadband
				GPIO.output(motor_pins[0],GPIO.LOW)
				GPIO.output(motor_pins[1],GPIO.LOW)
			elif self.V_cmd[i] > 0: # pos vels -> fwd
				GPIO.output(motor_pins[0],GPIO.HIGH)
				GPIO.output(motor_pins[1],GPIO.LOW)
				self.v_cmders[i].ChangeDutyCycle(max(OP_LIMIT,abs(self.V_cmd[i])))
			elif self.V_cmd[i] < 0: # neg vels -> bwd
				GPIO.output(motor_pins[0],GPIO.LOW)
				GPIO.output(motor_pins[1],GPIO.HIGH)
				self.v_cmders[i].ChangeDutyCycle(max(OP_LIMIT,abs(self.V_cmd[i])))

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown():
			interface.cmdMotors()
			rate.sleep()


def keyboardInterruptHandler(signal, frame):
	print("\nGPIO cleaning...")
	GPIO.cleanup()


if __name__ == '__main__':
	signal.signal(signal.SIGINT, keyboardInterruptHandler)
	interface = MotorInterface()
	interface.run()



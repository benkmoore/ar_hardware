
from sippy import functionset as fset
from sippy import *
import numpy as np
import control.matlab as cnt
import matplotlib.pyplot as plt

def armax(order, delay):
	sys_tf = system_identification(y, u, 'ARMAX', centering='MeanVal',
		ARMAX_orders=[order, order, order, delay], tsample=1, ARMAX_max_iterations=10)
	print('ARMAX - order: {}, delay: {} \n'.format(order, delay))
	print('Plant tf G: {} \n'.format(sys_tf.G))
	print('Error tf H: {} \n'.format(sys_tf.H))
	print('Model fit Error norm: {}'.format(sys_tf.Vn))
	print('MSE model predicted data to real data: {}\n'.format(fset.mean_square_error(sys_tf.Yid.T, y)))

raw_data = np.genfromtxt('output_DCMotor.csv', delimiter=';')

y = np.diff(raw_data[:,0]) 			# Get velocity steps/ms from encoder position
y = np.insert(y, 0, 0, axis=0)
y = y.reshape(-1,1)
u = raw_data[:,1]					# Input velocity command
t = raw_data[:,2] 					# ms

armax(order=1, delay=0)
armax(order=2, delay=0)
armax(order=1, delay=1)
armax(order=2, delay=1)

# plt.figure(1)
# plt.plot(t, y)
# plt.plot(t, sys_tf.Yid.T)
# plt.xlabel("Time")
# plt.title("Predicted out vs real out")
# plt.legend(['y real', 'y pred'])
# plt.grid()
# plt.show()

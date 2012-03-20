#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import rospy
from fmMsgs.msg import *

from math import *

class matrix:
    
	# implements basic operations of a matrix class

	def __init__(self, value):
		self.value = value
		self.dimx = len(value)
		self.dimy = len(value[0])
		if value == [[]]:
			self.dimx = 0
	
	def zero(self, dimx, dimy):
		# check if valid dimensions
		if dimx < 1 or dimy < 1:
			raise ValueError, "Invalid size of matrix"
		else:
			self.dimx = dimx
			self.dimy = dimy
			self.value = [[0 for row in range(dimy)] for col in range(dimx)] 
	
	def identity(self, dim):
		# check if valid dimension
		if dim < 1:
			raise ValueError, "Invalid size of matrix"
		else:
			self.dimx = dim
			self.dimy = dim
			self.value = [[0 for row in range(dim)] for col in range(dim)]
			for i in range(dim):
				self.value[i][i] = 1

	def show(self):
		for i in range(self.dimx):
			print self.value[i]
		print ' '

	def __add__(self, other):
		# check if correct dimensions
		if self.dimx != other.dimx or self.dimy != other.dimy:
			raise ValueError, "Matrices must be of equal dimensions to add"
		else:
			# add if correct dimensions
			res = matrix([[]])
			res.zero(self.dimx, self.dimy)
			for i in range(self.dimx):
				for j in range(self.dimy):
					res.value[i][j] = self.value[i][j] + other.value[i][j]
			return res

	def __sub__(self, other):
		# check if correct dimensions
		if self.dimx != other.dimx or self.dimy != other.dimy:
			raise ValueError, "Matrices must be of equal dimensions to subtract"
		else:
			# subtract if correct dimensions
			res = matrix([[]])
			res.zero(self.dimx, self.dimy)
			for i in range(self.dimx):
				for j in range(self.dimy):
					res.value[i][j] = self.value[i][j] - other.value[i][j]
			return res

	def __mul__(self, other):
		# check if correct dimensions
		if self.dimy != other.dimx:
			raise ValueError, "Matrices must be m*n and n*p to multiply"
		else:
			# subtract if correct dimensions
			res = matrix([[]])
			res.zero(self.dimx, other.dimy)
			for i in range(self.dimx):
				for j in range(other.dimy):
					for k in range(self.dimy):
						res.value[i][j] += self.value[i][k] * other.value[k][j]
			return res

	def transpose(self):
		# compute transpose
		res = matrix([[]])
		res.zero(self.dimy, self.dimx)
		for i in range(self.dimx):
			for j in range(self.dimy):
				res.value[j][i] = self.value[i][j]
		return res

	# Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

	def Cholesky(self, ztol=1.0e-5):
		# Computes the upper triangular Cholesky factorization of
		# a positive definite matrix.
		res = matrix([[]])
		res.zero(self.dimx, self.dimx)
		
		for i in range(self.dimx):
			S = sum([(res.value[k][i])**2 for k in range(i)])
			d = self.value[i][i] - S
			if abs(d) < ztol:
				res.value[i][i] = 0.0
			else:
				if d < 0.0:
					raise ValueError, "Matrix not positive-definite"
				res.value[i][i] = sqrt(d)
			for j in range(i+1, self.dimx):
				S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
				if abs(S) < ztol:
					S = 0.0
				res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
		return res
	
	def CholeskyInverse(self):
		# Computes inverse of matrix given its Cholesky upper Triangular
		# decomposition of matrix.
		res = matrix([[]])
		res.zero(self.dimx, self.dimx)
		
		# Backward step for inverse.
		for j in reversed(range(self.dimx)):
			tjj = self.value[j][j]
			S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
			res.value[j][j] = 1.0/tjj**2 - S/tjj
			for i in reversed(range(j)):
				res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
		return res

	def inverse(self):
		aux = self.Cholesky()
		res = aux.CholeskyInverse()
		return res

	def __repr__(self):
		return repr(self.value)





acc_data =  [0.,0.,0.]
gyro_data = [0.,0.,0.]
comp_data = [0.,0.,0.]

dt = 10;

F = matrix([[1., 0.], [0, 1.]]) # next state function
B = matrix([[dt, 0.], [0, dt]]) # control-input-coeff.
x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
H = matrix([[1., 0.], [0., 1.]]) # measurement function
R = matrix([[0.1, 0.], [0., 0.1]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

def filter():
    global F, B, x, P, u, H, R, I, comp_data, acc_data, gyro_data, dt
    print 'Start filter:'
    
    u = matrix([[acc_data[0]], [gyro_data[2]]])
    
    Z = matrix([[0., comp_data[2]]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - K * H) * P
    
    # prediction
    x = F * x + B * u
    P = F * P * F.transpose()
    
    print 'x= '
    x.show()
    print 'P= '
    P.show()


def acc_callback(data):
    global acc_data
    acc_data = [data.x, data.y, data.z]
    
def gyro_callback(data):
    global gyro_data
    gyro_data = [data.x, data.y, data.z]

def comp_callback(data):
    global comp_data
    comp_data = [data.x, data.y, data.z]

def timer_callback(event):
    rospy.logwarn("Timer times!")
    filter()
        
def kalman_main():
    global dt
    rospy.init_node('kalman_main')
    
    sub_accelerometer_topic_id  = rospy.get_param('~sub_accelerometer_topic_id' , "/default/Accelerometer")
    sub_gyroscope_topic_id = rospy.get_param('~sub_gyroscope_topic_id' , "/default/Gyroscope")
    sub_magnetometer_topic_id = rospy.get_param('~sub_magnetometer_topic_id' , "/default/Magnetometer")
    dt = float(rospy.get_param('~dt', 1.0))
    
    rospy.Subscriber(sub_accelerometer_topic_id, accelerometer, acc_callback)
    rospy.Subscriber(sub_gyroscope_topic_id, gyroscope, gyro_callback)
    rospy.Subscriber(sub_magnetometer_topic_id, magnetometer, comp_callback)
    
    rospy.Timer(rospy.rostime.Duration(dt), timer_callback)
    
    rospy.spin()

            
if __name__ == '__main__':
    try:
        kalman_main()
        
    except rospy.ROSInterruptException: pass
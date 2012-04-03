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






dt = 10;

# Inputs
gyro_input = matrix([[0.], [0.], [0.]])
acc_input = matrix([[0.], [0.], [0.]])
mag_input =  matrix([[0.], [0.], [0.]])

# Roll / Pitch
xhat = matrix([[0.], [0.]])
P = matrix([[1000.], [1000.]])

def pitch_and_roll():
    global gyro_input
    global xhat
    global P
    R = matrix([[1.], [1.], [1.]])
    
    pitch = rp_x.value[1][0] #Tall ø
    roll = rp_x.value[0][0]
    
    wx = gyro_input.value[0][0]
    wy = gyro_input.value[1][0]
    wz = gyro_input.value[2][0]
    
    tanpitch = tan(pitch)
    sinpitch = sin(pitch)
    cospitch = cos(pitch)
    sinroll = sin(roll)
    cosroll = cos(roll)
    
    fxu = matrix([[wx + tanpitch * sinroll * wy + tanpitch * cosroll * wz], 
                [cosroll * wy - sinroll * wz]])
    
    Axu = matrix([    [(wy * tanpitch * cosroll - wz * tanpitch * sinroll),   ((wy * sinroll + wz * cosroll) / cos(pitch)**2.)], 
                    [wy * sinroll - wz * cosroll,                           0.]])
    
    Q = matrix([[1.], [1.]])
    
    Wx = matrix([[1., tanpitch * sinroll, tanpitch * cosroll], 
                 [0., cosroll, - sinroll]])
    
                    
    # Time update
    xthat = xhat + fxu
    Pt = Axu * P * Axu.transpose() + Wx * Q * Wx.transpose()
    
    
    # Measurement update
    
    Hx = matrix([[0., cospitch], 
                 [- cosroll * cospitch, sinroll * sinpitch], 
                 [sinroll * cospitch, cosroll * sinpitch]])
                 
    Vx = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    
    hx = matrix([[sinpitch], 
                 [- sinroll * cospitch], 
                 [- cosroll * cospitch]])
    
    
    K = Pt * Hx.transpose() * (Hx * Pt * Hx.transpose() + Vx * R * Vx.transpose())
    
    xhat = xthat + k * ()
    
    #                 

    #             

    #             
    # 
    # 
    # Wx = matrix([[1., tanpitch * sinroll, tanpitch * cosroll], 
    #             [0., cosroll, - sinroll]])
                
    

def acc_callback(data):
    global acc_input
    acc_input = matrix([[float(data.x)], [float(data.y)], [float(data.z)]])
    
def gyro_callback(data):
    global gyro_input
    gyro_input = matrix([[float(data.x)], [float(data.y)], [float(data.z)]])

def comp_callback(data):
    global mag_input
    mag_input = matrix([[float(data.x)], [float(data.y)], [float(data.z)]])

def timer_callback(event):
    pitch_and_roll()
    #rp_x.show()
    A = matrix([[1, 2], [3, 4], [5, 6]])
    A.show()
    rospy.logwarn("Timer times!")
        
def main():
    global dt
    rospy.init_node('main')
    sub_accelerometer_topic_id  = rospy.get_param('~sub_accelerometer_topic_id' , "/default/Accelerometer")
    sub_gyroscope_topic_id = rospy.get_param('~sub_gyroscope_topic_id' , "/fmSensors/Gyroscope")
    sub_magnetometer_topic_id = rospy.get_param('~sub_magnetometer_topic_id' , "/default/Magnetometer")
    dt = float(rospy.get_param('~dt', 1.0))
    
    rospy.Subscriber(sub_accelerometer_topic_id, accelerometer, acc_callback)
    rospy.Subscriber(sub_gyroscope_topic_id, gyroscope, gyro_callback)
    rospy.Subscriber(sub_magnetometer_topic_id, magnetometer, comp_callback)
    
    rospy.Timer(rospy.rostime.Duration(dt), timer_callback)
    
    rospy.spin()

            
if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException: pass
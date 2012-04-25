#! /usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy
from fmMsgs.msg import float_data, row, gyroscope
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import tf

from math import *
import matrix

class row_localizer:
    """The row Localizer"""
    def  __init__(self):
        # initialize global variables
        
        self.x = 0                  # state variables
        self.y = 0
        self.th = 0
        
        self.dt = 0.02              # discrete time step
        self.b = (2 * 0.185)        # Distance between wheels
        self.threshold = 0.0001     # threshold value for angle calculation 
        
        self.vl = 0                 # Left speed
        self.vr = 0                 # Right speed
        self.new_speeds = 0         # Detect new speeds
        
        self.gyro_z = 0             # gyro z         
        self.gyro_z_first = 100000  # Off set gyro
        self.new_gyro = 0           # Detect new gyro data
         
        self.row_right_angle = 0    # Row variables
        self.row_right_dist = 0
        self.row_left_angle = 0
        self.row_left_dist = 0
        self.new_row_right = 0           # detect new rows
        self.new_row_left = 0
        
        # Weight between encoder and gyro 
        self.angle_gain = 0.2
        # KALMAN
        self.F = matrix.matrix([[1., self.dt*(1 - self.angle_gain), self.dt*self.angle_gain], [0., 1., 0.], [0., 0., 1.]]) # next state function
        self.kx = matrix.matrix([[0.], [0.], [0.]]) # initial state (location and velocity)
        self.P = matrix.matrix([[1000., 0., 0.], [0., 1000., 0.], [0., 0., 1000.]]) # initial uncertainty
        self.H = matrix.matrix([[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]]) # measurement function
        self.R = matrix.matrix([[0.5, 0., 0.], [0., 0.5, 0.], [0., 0., 0.5]]) # measurement uncertainty
        self.I = matrix.matrix([[1., 0., 0.], [0., 1., 0], [0., 0., 1]]) # identity matrix

         
        rospy.init_node('row_localizer')
        rospy.loginfo("Localizing is on")
        rospy.Subscriber("/Hilde/wheel_speed_topic", float_data, self.wheel_speed_callback)
        rospy.Subscriber("/lrs/row_topic", row, self.row_callback)
        rospy.Subscriber("/fmSensors/Gyroscope", gyroscope, self.gyro_callback)
        
        self.odom_pub = rospy.Publisher("/base_odom", Odometry)
        
        rospy.Timer(rospy.rostime.Duration(self.dt), self.timer_callback)
        rospy.spin()
        
    def gyro_callback(self, gyro):
        # Declare the gyro offset
        if self.gyro_z_first == 100000:
            self.gyro_z_first = self.gyro_z
            print self.gyro_z_first
        
        self.gyro_z = gyro.z
        self.new_gyro = 1
        #rospy.loginfo("Gyro callback %f"%gyro.z)
        
    def row_callback(self, rows):
        self.row_left_angle = rows.leftangle
        self.row_right_angle = rows.rightangle
        self.row_left_dist = rows.leftdistance
        self.row_right_dist = rows.rightangle
        self.new_row_right = rows.rightvalid
        self.new_row_left = rows.leftvalid
        #rospy.loginfo("row callback")
        
    def wheel_speed_callback(self, speeds):
        self.vl = speeds.data[1] # Left speed
        self.vr = speeds.data[0] # Right speed
        self.new_speeds = 1
        #rospy.loginfo("Wheel callback")
    
    def timer_callback(self, event):
        
        # if new values are detected
        if self.new_gyro == 1 and self.new_speeds == 1:
            wt = (self.vr - self.vl)/self.b        # Angular velocity from encoders    - 50 Hz
            self.gyro_z_first = - 0.02
            wg = self.gyro_z - self.gyro_z_first    # Angular velocity from gyro - 100 Hz and an offset
            # if left row is received
            if self.new_row_left:
                Z = matrix.matrix([[self.row_left_angle, wt, wg]])
                self.H.value[0][0] = 1
            Z = matrix.matrix([[0., wt, wg]])
            self.filter(Z)
            values = self.kx.value
            self.th = values[0][0]
            self.locate(values[1][0], values[2][0])
            self.new_gyro = 0
            self.new_speeds = 0
            
        #rospy.loginfo("Timer")
    def locate(self, wt, wg):
        #rospy.loginfo("Calculating")
        # Start locating
        V = (self.vl + self.vr)/2              # Linear velocity from encoder
        #wg = self.gyro_z + 0.05
        w = wt * (1 - self.angle_gain) + wg * (self.angle_gain)                     # Angular velocity combined
        #print "ticks/gyro: ", wt, " ", wg
        beta = w * self.dt
        dist = V * self.dt
        #print 'V: ', V, " w: ", w, " beta: ", beta, " dist: ", dist
        x = self.x
        y = self.y
        O = self.th
        
        if abs(beta) < self.threshold:
            x = x + dist*cos(O)
            y = y + dist*sin(O)
            O = (O + beta) % (2*pi)
        else:
            Cr = dist/beta
            Cx = x - sin(O)*Cr
            Cy = y + cos(O)*Cr
            
            x = Cx + sin(O + beta)*Cr
            y = Cy - cos(O + beta)*Cr
            O = (O + beta) % (2*pi)
        #print "x: ", x, " y: ", y, " th: ", O
        self.x = x
        self.y = y
        #self.th = O
        
        # Publish Odom msg
        odom_msg = Odometry()
        #odom_msg.header.stamp = rospy.get_time()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0,0,O)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        #print quat
        self.odom_pub.publish(odom_msg) 

    def filter(self, Z):
        #print 'Start filter:'
        
        H = self.H
        x = self.kx
        P = self.P
        R = self.R
        I = self.I
        F = self.F
        
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - K * H) * P
        
         # prediction
        x = F * x
        P = F * P * F.transpose()
        
        print 'x= '
        x.show()
        print 'P= '
        P.show()
        
        self.kx = x
        self.P = P

# Main loop
if __name__ == '__main__':
    row_localizer();

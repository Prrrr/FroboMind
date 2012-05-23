#! /usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy
from fmMsgs.msg import float_data, row, gyroscope, hilde_states
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import tf

import pylab

from math import *
import matrix

class row_localizer:
    """The row Localizer"""
    def  __init__(self):
        # ROS
        rospy.init_node('row_localizer')
        wheel_speed_topic = rospy.get_param('~wheel_speed_topic', "/Hilde/wheel_speed_topic")
        row_topic = rospy.get_param('~row_topic', "/lrs/row_topic")
        gyro_topic = rospy.get_param('~gyro_topic', "/fmSensors/Gyroscope")
        odom_topic = rospy.get_param('~odom_topic', "/base_odom")
        row_state_topic = rospy.get_param('~row_state_topic', "/nav/row_state_topic")
        
        self.frame_id = rospy.get_param('~frame_id', "base")
        self.child_frame_id = rospy.get_param('~child_frame_id', "map")
        self.dt = float(rospy.get_param('~dt', 0.02))
        rospy.loginfo("child: %s"%self.child_frame_id)
        rospy.loginfo("frame: %s"%self.frame_id)
        
        rospy.Subscriber(wheel_speed_topic, float_data, self.wheel_speed_callback)
        rospy.Subscriber(row_topic, row, self.row_callback)
        rospy.Subscriber(gyro_topic, gyroscope, self.gyro_callback)
        rospy.Subscriber(row_state_topic, hilde_states, self.row_state_callback)
        
        self.odom_pub = rospy.Publisher(odom_topic, Odometry)
        
        rospy.Timer(rospy.rostime.Duration(self.dt), self.timer_callback)
        
        # initialize global variables
        
        self.x = 0                  # state variables
        self.y = 0
        self.th = 0
        
        self.x_list = []            # lists for saving values
        self.y_list = []
        self.th_list = []
        
        self.b = (2 * 0.185)        # Distance between wheels
        self.threshold = 0.0001     # threshold value for angle calculation 
        
        self.vl = 0                 # Left speed
        self.vr = 0                 # Right speed
        self.new_speeds = 0         # Detect new speeds
        
        self.gyro_z = 0             # gyro z         
        self.gyro_z_first = 100000  # Off set gyro
        self.new_gyro = 0           # Detect new gyro data
        self.gyro_list = [0.0 for i in range(50)]   # list for off set gyro data
         
        self.row_right_angle = 0    # Row variables
        self.row_right_dist = 0
        self.row_left_angle = 0
        self.row_left_dist = 0
        self.new_row_right = 0           # detect new rows
        self.new_row_left = 0
        
        # Row states
        self.RST_BETWEEN_ROWS    = 0
        self.RST_RIGHT_ROW       = 1
        self.RST_LEFT_ROW        = 2
        self.RST_NO_ROW          = 3
        self.row_state = self.RST_NO_ROW
    
        # Weight between encoder and gyro 
        self.angle_gain = 0
        # KALMAN
        self.F = matrix.matrix([[1., self.dt*(1 - self.angle_gain), self.dt*self.angle_gain], 
                                [0., 1., 0.], 
                                [0., 0., 1.]]) # next state function
        self.kx = matrix.matrix([[0.], [0.], [0.]]) # initial state (location and velocity)
        self.P = matrix.matrix([[1000., 0., 0.], 
                                [0., 1000., 0.], 
                                [0., 0., 1000.]]) # initial uncertainty
        self.H = matrix.matrix([[0., 0., 0.], 
                                [0., 1., 0.], 
                                [0., 0., 1.]]) # measurement function
        self.R = matrix.matrix([[0.1, 0., 0.], 
                                [0., 0.5, 0.], 
                                [0., 0., 0.9]]) # measurement uncertainty
        self.I = matrix.matrix([[1., 0., 0.], 
                                [0., 1., 0], 
                                [0., 0., 1]]) # identity matrix

        rospy.loginfo("Localization is on")
        # ROS
        rospy.spin()
    
    def row_state_callback(self, state):
        self.row_state = state.state
        
    def gyro_callback(self, gyro):
        # Declare the gyro offset
        if self.vl == 0 and self.vr == 0:
            self.gyro_z = 0
            self.gyro_list.pop(0)
            self.gyro_list.append(gyro.z)
            self.gyro_z_first = sum(self.gyro_list) / len(self.gyro_list)
            print self.gyro_z_first
        else:
            self.gyro_z = gyro.z - self.gyro_z_first
            #self.gyro_z = gyro.z
        
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
            wg = self.gyro_z    # Angular velocity from gyro - 100 Hz
            # reset
            Z = matrix.matrix([[0., wt, wg]])
            self.H.value[0][0] = 0
        
            # Check the row states
#            if self.row_state == self.RST_LEFT_ROW:
#                # if left row is received
#                if self.new_row_left:
#                    Z = matrix.matrix([[self.row_left_angle, wt, wg]])
#                    self.H.value[0][0] = 1
#                    self.new_row_left = 0
#            elif self.row_state == self.RST_RIGHT_ROW:
#                # if right row is received
#                if self.new_row_right:
#                    Z = matrix.matrix([[self.row_right_angle, wt, wg]])
#                    self.H.value[0][0] = 1
#                    self.new_row_right = 0
#            elif self.row_state == self.RST_BETWEEN_ROWS:
#                angle = 0
#                angle_cnt = 0
#                # if right is received
#                if self.new_row_right:
#                    angle += self.row_right_angle
#                    angle_cnt += 1
#                    self.new_row_right = 0
#                # if left is received
#                if self.new_row_left:
#                    angle += self.row_left_angle
#                    angle_cnt += 1 
#                    self.new_row_left = 0
#                
#                # use the angles
#                if angle_cnt > 0:
#                    angle /= angle_cnt     
#                    Z = matrix.matrix([[angle, wt, wg]])
#                    self.H.value[0][0] = 1
#                    
#            else: #             self.row_state == self.RST_NO_ROW:
#                Z = matrix.matrix([[0., wt, wg]])
#                self.H.value[0][0] = 0
            
            # For Kalman
#            self.filter(Z)
#            values = self.kx.value
#            self.th = values[0][0]
#            self.locate(values[1][0], values[2][0])


            # For tests without kalman
            angck = self.th
            if self.row_state == self.RST_LEFT_ROW:
                # if left row is received
                if self.new_row_left:
                    if self.th > pi / 2:
                        angck = pi - self.row_left_angle
                    elif self.th < - (pi / 2):
                        angck = - (pi + self.row_left_angle)
                    else:
                        angck = self.row_left_angle
                    self.new_row_left = 0
            elif self.row_state == self.RST_RIGHT_ROW:
                # if right row is received
                if self.new_row_right:
                    if self.th > pi / 2:
                        angck = pi - self.row_right_angle
                    elif self.th < - (pi / 2):
                        angck = - (pi + self.row_right_angle)
                    else:
                        angck = self.row_right_angle            
                    self.new_row_right = 0
            elif self.row_state == self.RST_BETWEEN_ROWS:
                angle = 0
                angle_cnt = 0
                # if right is received
                if self.new_row_right:
                    if self.th > pi / 2:
                        angle += pi - self.row_right_angle
                    elif self.th < - (pi / 2):
                        angle += - (pi + self.row_right_angle)
                    else:
                        angle += self.row_right_angle
                    angle_cnt += 1
                    self.new_row_right = 0
                # if left is received
                if self.new_row_left:
                    if self.th > pi / 2:
                        angle += pi - self.row_left_angle
                    elif self.th < - (pi / 2):
                        angle += - (pi + self.row_left_angle)
                    else:
                        angle += self.row_left_angle
                    angle_cnt += 1 
                    self.new_row_left = 0
                
                # use the angles
                if angle_cnt > 0:
                    angle /= angle_cnt     
                    angck = angle
                    
            else: #             self.row_state == self.RST_NO_ROW:
                angck = self.th
            
            self.th = angck + wt * self.dt
            #print "th " + str(self.th)
            self.locate(wt, wg)
            # save the values
            self.x_list.append(self.x)
            self.y_list.append(self.y)
            self.th_list.append(self.th)
            # approx. 3800 items in list
            #print len(self.x_list)
            if  len(self.x_list) >= 3000:
                pylab.figure()
                pylab.subplot(211)
                pylab.plot(self.x_list, self.y_list)
                pylab.xlabel('x / [m]')
                pylab.ylabel('y / [m]')
                pylab.legend()
                pylab.subplot(212)
                t_list = [i for i in range(len(self.th_list))]
                pylab.plot(t_list, self.th_list)
                pylab.xlabel('time / [?]')
                pylab.ylabel('Theta / [rad]')
                pylab.legend()
                pylab.show()
            # reset values
            self.new_gyro = 0
            self.new_speeds = 0
            self.H.value[0][0] = 0
            
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
        
        # Transform etc
        br = tf.TransformBroadcaster()
        quat = tf.transformations.quaternion_from_euler(0,0,O)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = self.frame_id
        odom_trans.child_frame_id = self.child_frame_id
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = quat;
        #tf.TransformBroadcaster().SendTransform(R,t,time,child,parent)
        br.sendTransform((x,y,0.0), quat, odom_trans.header.stamp, odom_trans.child_frame_id, odom_trans.header.frame_id)
        
        # Publish Odom msg
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
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
        
        #print 'x= '
        #x.show()
        #print 'P= '
        #P.show()
        
        self.kx = x
        self.P = P

# Main loop
if __name__ == '__main__':
    try:
        row_localizer();
    except rospy.ROSInterruptException: pass
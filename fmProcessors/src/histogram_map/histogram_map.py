#! /usr/bin/env python
import roslib; roslib.load_manifest('fmProcessors')
import rospy
from fmMsgs.msg import float_data
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
import math
import tf
import random

from histogram_map import *
from sensor_msgs.msg._PointCloud import PointCloud

DEF_VISUALIZE = 1
dt = 0.1
class Map:
    def __init__(self, x_meter, y_meter, res = 10):
        rospy.loginfo("Map started")
        self.init_map(x_meter, y_meter, res)

    def init_map(self, x_meter, y_meter, res):
        self.xm = x_meter   # map width in m
        self.ym = y_meter   # map height in m        
        self.resolution = res          # resolution
        self.x_cells = int(self.xm / self.resolution)
        self.y_cells = int(self.ym / self.resolution)
        #self.map = [[0 for y in range(self.y_cells)] for x in range(self.x_cells)]
        self.map = [[0 for x in range(self.x_cells)] for y in range(self.y_cells)]
        
        self.offset_x = 2       # offset to the first row in m
        self.offset_y = 1       # headland in m
        self.row_width = 0.40    # width of the row
        self.row_spacing = 0.75       # spacing between rows
        self.row_height = int(self.ym - (2*self.offset_y))   # Row is as long as the map minus the headlands
        
    
    def draw_row(self, number):
        cell_start_x = int((self.offset_x + (number*self.row_spacing+number*self.row_width)) 
                           / self.resolution)
        cell_start_y = int(self.offset_y / self.resolution)
        cell_amount_length = int(self.row_height / self.resolution)
        cell_amount_width = int(self.row_width / self.resolution)
        gradient = int(0.2 / self.resolution)
        
        
        for y in range(cell_amount_length):
            for x in range(cell_amount_width):
                this_y = cell_start_y + y
                this_x = cell_start_x + x
                center = cell_start_x + gradient 
                weight =  float((this_x - center))/float(gradient)
                #print 100 * abs(weight) 
                if abs(weight) >= 0.5:
                    self.map[this_y][this_x] = 100
                else:
                    if this_y > (cell_start_y + gradient/2) and this_y < (cell_start_y + cell_amount_length - gradient/2):
                        self.map[this_y][this_x] = 1
                    else:   # draw ends of row
                        self.map[this_y][this_x] = 100
                # draw veil
                amount = int(cell_amount_width/10)
                if (this_x + cell_amount_width) < (cell_start_x + cell_amount_width + amount):
                     self.map[this_y][this_x + cell_amount_width] = int(100/(x+1))
                this_x = cell_start_x - x
                if this_x > (cell_start_x - amount):   
                    self.map[this_y][this_x] = int(100/(x+1))
                
                
                
                     
         
#    def put_xy_box(self, x_start, y_start, size):
#        cell_start_x = int(x_start / self.resolution)
#        cell_start_y = int(y_start / self.resolution)
#        for y in range(int(size/self.resolution)):
#            for x in range(int(size/self.resolution)):
#                if y > 0 and x > 0 and y < self.y_cells and x < self.x_cells:
#                    self.map[cell_start_y + y][cell_start_x + x] = 1
#                
#    def print_map(self):
#        for i in range(self.y_cells):
#            print self.map[i]
#            
#    def print_partial_map(self, x_size, y_size, offset_x = 0, offset_y = 0):
#        for i in range(y_size):
#            print self.map[i+offset_y][offset_x:x_size]

class Robot:
    def __init__(self):
        y = random.random()*0.2 + 1
        x = random.random()*1 + 2.40
        th = random.random()*0.5 + math.pi/2
        self.w = 1.0
        self.set(x, y, th)
        self.set_noise(0.1, 0.1, 0.1)
        
    def set(self, x, y, th):
        self.b = 0.19
        self.x = x
        self.y = y
        self.th = th
    
    # --------
    # set_noise: 
    #    sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.th))
    def move(self, left, right, dt):
        V = (left + right)/2
        V += random.gauss(0.0, self.steering_noise)
        w = (right - left)/(2*self.b)
        w += random.gauss(0.0, self.bearing_noise)
        x = self.x + V*math.cos(self.th)*dt
        y = self.y + V*math.sin(self.th)*dt
        th = (self.th + w*dt) % (2*math.pi)
        # create new to make sure the are different
        res = Robot()
        res.set(x,y,th)
        return res


markerArray = MarkerArray()
point_cloud = PointCloud()                  
class HistogramLoc:
    def __init__(self):
        print "Localization initialized"
        self.wheel_speed_left = 0
        self.wheel_speed_right = 0
        self.robot = Robot()
        self.pf = ParticleFilter(100)
          
        rospy.init_node('histogram_localization')
        self.odomPub = rospy.Publisher("/base_odom", Odometry)
        self.mapPub = rospy.Publisher("/myMap", OccupancyGrid)
        
        rospy.Subscriber("/Hilde/wheel_speed_topic", float_data, self.wheel_speed_callback)
        rospy.Subscriber("/lrs/laser_msgs_1", LaserScan, self.scan_callback)
        rospy.Timer(rospy.rostime.Duration(dt), self.timer_callback)
        
        if DEF_VISUALIZE == 1:  # This is onlu for visualization
            self.markerArrayPub = rospy.Publisher("/marker_array", MarkerArray) 
            self.pointCloudPub = rospy.Publisher("/point_cloud", PointCloud)
            self.publish_map()
        # ROS
        rospy.spin()
    
    def scan_callback(self, scan):
        global point_cloud
        # project the laser beams into x and y
        pc = PointCloud()
        start_angle = scan.angle_min
        stop_angle = scan.angle_max
        delta_angle = scan.angle_increment
        for i in range(len(scan.ranges)):
            angle = start_angle +i*delta_angle
            # inverted ?
            #angle = stop_angle - i*delta_angle
            point = Point32()
            point.x = scan.ranges[i]*math.cos(angle)
            point.y = scan.ranges[i]*math.sin(angle)
            point.z = 0
            pc.points.append(point)
        # copy over 
        point_cloud = pc
        if DEF_VISUALIZE == 1:
            # publish pointcloud
            point_cloud.header.stamp = rospy.Time.now()
            point_cloud.header.frame_id = "/base_link"
            self.pointCloudPub.publish(point_cloud)
    def wheel_speed_callback(self, speeds):
        self.wheel_speed_right = speeds.data[0]
        self.wheel_speed_left = speeds.data[1]

    def timer_callback(self, event):
        #rospy.loginfo("Timer callback left: %f" %self.wheel_speed_left)
        #self.robot.move(0.05, 0.05, self.dt)
        self.pf.left_wheel = self.wheel_speed_left
        self.pf.right_wheel = self.wheel_speed_right
        robot = self.pf.run_filter()
        self.publish_odom(robot)
        if DEF_VISUALIZE == 1:
            self.markerArrayPub.publish(markerArray)
            self.publish_map()
        #print "pos: ", pos
            
    def publish_odom(self, robot):
        # Transform etc
        x = robot.x
        y = robot.y
        th = robot.th
        br = tf.TransformBroadcaster()
        quat = tf.transformations.quaternion_from_euler(0,0,th)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "/map"
        odom_trans.child_frame_id = "/base_link"
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = quat;
        #tf.TransformBroadcaster().SendTransform(R,t,time,child,parent)
        br.sendTransform((x,y,0.0), quat, odom_trans.header.stamp, odom_trans.child_frame_id, odom_trans.header.frame_id)
        
        # Publish Odom msg
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/map"
        odom_msg.child_frame_id = "/base_link"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # publish
        self.odomPub.publish(odom_msg)
        # print
        #rospy.loginfo("Odom: %f %f" %(x,y))
        
    def publish_map(self):
        if DEF_VISUALIZE == 1:
            grid = OccupancyGrid()
            grid.header.stamp = rospy.Time.now()
            grid.info.height = self.pf.mymap.y_cells
            grid.info.width = self.pf.mymap.x_cells
            grid.header.frame_id = "/map"
            grid.info.resolution = self.pf.mymap.resolution
            grid.info.map_load_time = rospy.Time.now()
            for y in range(grid.info.height):
                for x in range(grid.info.width):
                    grid.data.append(self.pf.mymap.map[y][x])
            self.mapPub.publish(grid)

class ParticleFilter:
    def __init__(self, N = 100):
        # Create a map
        self.mymap = Map(10, 7, 0.01)
        for i in range(5):
            self.mymap.draw_row(i)
        # pu in random box to test direction
        #self.mymap.put_xy_box(5, 0, 0.2)

        self.left_wheel = 0
        self.right_wheel = 0

        # make particles
        self.p = []
        for i in range(N):
            r = Robot()
            self.p.append(r)
            #print r
            
    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    def get_position(self, p):
        x = 0.0
        y = 0.0
        orientation = 0.0
        for i in range(len(p)):
            x += p[i].x
            y += p[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((p[i].th - p[0].th + math.pi) % (2.0 * math.pi)) 
                            + p[0].th - math.pi)
        #return [x / len(p), y / len(p), orientation / len(p)]
        robot = Robot()
        robot.set(x / len(p), y / len(p), orientation / len(p))
        return robot
    
    def run_filter(self):
        N = len(self.p)
        p = self.p
        # move
        p2 = []
        for i in range(N):
            p2.append(p[i].move(self.left_wheel, self.right_wheel,dt))
        p = p2
        # measurement update
        w = []
        self.update()
        for i in range(len(self.p)):
            w.append(self.p[i].w)
            print "#:", i, " : ", self.p[i].w
        
        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
            
#        for i in range(len(p)):
#            print p[i]
#        print " - "
        p = p3
#        for i in range(len(p)):
#            print p[i]
#        print " __________________________ "
        # publish marker arrays
        if DEF_VISUALIZE == 1:
            markerArray.markers = []
            for i in range(N):
                quat = tf.transformations.quaternion_from_euler(0,0,p[i].th)
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "/map"
                marker.ns = "particles"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = p[i].x
                marker.pose.position.y = p[i].y
                marker.pose.position.z = 0.0
                marker.pose.orientation = Quaternion(*quat) 
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.1
                marker.color.a = 0.7
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 1
                markerArray.markers.append(marker)
            
        self.p = p
        #print self.p[0].x, " ", p[0].y 
        return self.get_position(p)
    
    def update(self):
        #print "1: " , len(self.p)
        #print "1: " , len(point_cloud)
        for i in range(len(self.p)):
            self.p[i].w = 0
            for j in range(len(point_cloud.points)):
                # transpose laser scan into particle's view
                px = point_cloud.points[j].x * math.cos(self.p[i].th) - point_cloud.points[j].y * math.sin(self.p[i].th)
                py = point_cloud.points[j].x * math.sin(self.p[i].th) + point_cloud.points[j].y * math.cos(self.p[i].th)

                px += self.p[i].x
                py += self.p[i].y
                #print "px: " , px
                
                res = self.mymap.resolution
                x = int(px/res)
                y = int(py/res)
                map = self.mymap.map
                #print "x: ", x, " y:", y, " lenx: ", len(map), "leny: ", len(map[0])
                # calc distance to point
                dx = self.p[i].x - px
                dy = self.p[i].y - py
                dist = math.sqrt(dx**2 + dy**2)
                #print "distance: ", dist
                if x < len(map[0]) and x > 0 and y < len(map) and y > 0:
                    # check for hit
                    if not map[y][x] == 0:
                        #print "hit: ", self.p[i]
                        self.p[i].w += map[y][x]
                    
                # check for particle conflict with map
                x = int(self.p[i].x/res)
                y = int(self.p[i].y/res)
                if not map[y][x] == 0:
                    self.p[i].w = 0
                    


# Main loop
if __name__ == '__main__':
    try:
        HistogramLoc()
    except rospy.ROSInterruptException: pass
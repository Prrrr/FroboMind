#!/usr/bin/env python
import roslib; roslib.load_manifest('fmExtractors')
import rospy
from fmMsgs.msg import *

from math import *

dt = 10;

# Inputs
mag_input =  [0., 100., 0.]

def comp_callback(data):
    global mag_input
    mag_input = [float(data.x), float(data.y), float(data.z)]
    # if (float(data.x) > mag_input[0]):
    #     mag_input[0] = float(data.x)
    #     
    # if (float(data.x) < mag_input[1]):
    #     mag_input[1] = float(data.x)
    

def timer_callback(event):
    global mag_input
    
    # heading = 0
    #     
    #     if (mag_input[0] != 0):
    #         heading = atan2(mag_input[1], mag_input[0]) * (180 / 3.1415) - 90
    #         if (heading > 0):
    #             heading -= 360
    #         
    #         rospy.loginfo("Heading: " + str(heading))
    
    rospy.loginfo("Total: " + str(mag_input[0] + mag_input[1] + mag_input[2]))
    #rospy.loginfo("Min: " + str(mag_input[1]))
      
    rospy.logwarn("Timer times!")
        
def main():
    global dt
    rospy.init_node('main')
    
    sub_magnetometer_topic_id = rospy.get_param('~sub_magnetometer_topic_id' , "/default/Magnetometer")
    dt = float(rospy.get_param('~dt', 1.0))
    
    rospy.Subscriber(sub_magnetometer_topic_id, magnetometer, comp_callback)
    
    rospy.Timer(rospy.rostime.Duration(dt), timer_callback)
    
    rospy.spin()

            
if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException: pass
/*
 * SimpleObjectAvoidance.h
 *
 *  Created on: Apr 20, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef SIMPLEOBJECTAVOIDANCE_H_
#define SIMPLEOBJECTAVOIDANCE_H_
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "fmMsgs/detected_objects.h"
#include "fmMsgs/object_row.h"
#include "fmMsgs/float_data.h"

// OpenCv includes
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "math.h"
#include "vector"

using namespace std;

class SimpleObjectAvoidance {
private:
	IplImage* raw_img;
	IplImage* grey_img;
	// Image parameters
	const char* framedWindowName;

	// ROS
	// Nodehandle
	ros::NodeHandle n;
	ros::NodeHandle nh;
	// Subscribers & publishers
	ros::Subscriber laser_subscriber;
	ros::Publisher object_publisher;
	ros::Publisher object_row_publisher;
	ros::Subscriber wheel_speeds_subscriber;
	
	// msgs
	fmMsgs::detected_objects object_msg;
	fmMsgs::object_row object_row_msg;
	
	// Parameters
	string laser_scan_topic, object_topic, object_row_topic, wheel_speeds_topic;
	int show_image;
	double robot_clearence_width, robot_stop_zone, robot_turn_zone, robot_turn_zone_extra_width;
	
	// Rows of boxes
	double row_box_start_value, row_box_width, row_box_height, row_individual_box_height;
	int row_box_count;
	
	// Callbacks
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
	void wheelCallback(const fmMsgs::float_data::ConstPtr& speeds);
	
	// Wheel speeds
	int new_speeds;
	double wheel_speed_right, wheel_speed_left;
	
	// Detected rows
	// vector<int> left_row;
	// vector<int> right_row;
public:
	// Constructor & Destructor
	SimpleObjectAvoidance();
	virtual ~SimpleObjectAvoidance();

};

#endif /* SIMPLEOBJECTAVOIDANCE_H_ */

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

// OpenCv includes
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "math.h"
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
	// Subscribers & publishers
	ros::Subscriber laser_subscriber;
	// Parameters
	string laser_scan_topic;
	int show_image;
	double robot_clearence_width;
	// Callbacks
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
public:
	// Constructor & Destructor
	SimpleObjectAvoidance();
	virtual ~SimpleObjectAvoidance();

};

#endif /* SIMPLEOBJECTAVOIDANCE_H_ */

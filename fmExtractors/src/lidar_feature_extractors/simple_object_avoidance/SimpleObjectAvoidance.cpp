/*
 * SimpleObjectAvoidance.cpp
 *
 *  Created on: Apr 20, 2012
 *      Author: Morten Ege Jensen
 */

#include "SimpleObjectAvoidance.h"

SimpleObjectAvoidance::SimpleObjectAvoidance() {
	// ROS NodeHandles
	n = ros::NodeHandle();
	// ROS PARAMETERS
	n.param<string>("laser_scan_topic", laser_scan_topic, "laser_scan_topic");
	n.param<int>("show_image", show_image, 1);
	n.param<double>("robot_clearence_width", robot_clearence_width, 0.2);

	// ROS PUBLISHERS AND SUBSCRIBERS
	laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 1, &SimpleObjectAvoidance::laserScanCallback, this);

	// Create images
	raw_img = cvCreateImage(cvSize(600, 600), 8, 3);
	grey_img = cvCreateImage(cvSize(600, 600), 8, 1);
	// Create window handler
	if (show_image){
		// Create a window
		framedWindowName = "OpenCVWindowName";
		cvNamedWindow(framedWindowName);
		cvStartWindowThread();
		// Show image test
		//cvShowImage(framedWindowName, rawData_img);
	}
}

SimpleObjectAvoidance::~SimpleObjectAvoidance() {
	if (show_image){
		cvDestroyWindow(framedWindowName);
	}
}

void SimpleObjectAvoidance::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan){
	//ROS_INFO("Laser Scan Callback");
	// Clear image
	cvZero(raw_img);
	// Create PointCloud from laser scan
	sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection projector;
	projector.projectLaser(*laser_scan, cloud);

	// create img data from cloud data
	int size = cloud.points.size();
	CvPoint point1;
	for (int i = 0; i < size; i++) {
		if (abs((cloud.points[i].y))<10 && cloud.points[i].x < 10 ) {
			point1.x = ((int)(cloud.points[i].x * 100) + 300);
			point1.y = ((int)(cloud.points[i].y * 100) + 300);
			cvCircle(raw_img,point1,1,CV_RGB(255,255,255));	// Marks the point with a circle
		}
	}
	CvPoint recta, rectb;
	recta.x = 300;
	recta.y = 300 - robot_clearence_width * 100;
	rectb.x = 300 + 20;
	rectb.y = 300 - 1;
	cvRectangle(raw_img,recta, rectb, CV_RGB(255,0,0),1,1,0);
	recta.x = 300;
	recta.y = 300 + robot_clearence_width * 100;
	rectb.x = 300 + 20;
	rectb.y = 300 + 1;
	cvRectangle(raw_img,recta, rectb, CV_RGB(0,255,0),1,1,0);

	// Check quadrant without OpenCV
	double r, alpha, x, y;
	for (int i = 0; i < size; i++) {
		x = cloud.points[i].x;
		y = cloud.points[i].y;
		if (x > 0 && x < 0.2){
			if (y > 0 && y < robot_clearence_width) {
				// If there are objects in the RIGHT quadrant
				r = sqrt(x * x + y * y);
				alpha = atan2(y,x);
				ROS_INFO("RIGHT r: %f, ang: %f", r, alpha);
				point1.x = ((int)(cloud.points[i].x * 100) + 300);
				point1.y = ((int)(cloud.points[i].y * 100) + 300);
				cvCircle(raw_img,point1,1,CV_RGB(255,255,0));	// Marks the point with a circle
			}else if (y < 0 && y > -robot_clearence_width) {
				// If there are objects in the LEFT quadrant
				r = sqrt(x * x + y * y);
				alpha = atan2(y,x);
				ROS_INFO("LEFT r: %f, ang: %f", r, alpha);
				point1.x = ((int)(cloud.points[i].x * 100) + 300);
				point1.y = ((int)(cloud.points[i].y * 100) + 300);
				cvCircle(raw_img,point1,1,CV_RGB(0,255,255));	// Marks the point with a circle
			}
		}
	}


	cvShowImage(framedWindowName,raw_img);
}

int main(int argc, char** argv){
	// Initialize ros
	ros::init(argc, argv, "simple_object_avoidance");
	// Instantiate Object
	SimpleObjectAvoidance soa;
	// Spin
	ros::spin();
	return 0;
}

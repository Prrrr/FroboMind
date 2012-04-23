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
	n.param<double>("robot_stop_zone", robot_stop_zone, 0.2);
	n.param<double>("robot_turn_zone", robot_turn_zone, 0.3);
	n.param<double>("robot_turn_zone_extra_width", robot_turn_zone_extra_width, 0.05);
	n.param<string>("object_topic", object_topic, "object_topic");

	// For row boxes
	n.param<double>("row_box_start_value", row_box_start_value, -0.2);
	n.param<double>("row_box_height", row_box_height, 1);
	n.param<double>("row_box_width", row_box_width, 0.2);
	n.param<int>("row_box_count", row_box_count, 5);

	// ROS PUBLISHERS AND SUBSCRIBERS
	laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 1, &SimpleObjectAvoidance::laserScanCallback, this);
	object_publisher = n.advertise<fmMsgs::detected_objects>(object_topic.c_str(), 1);
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
	// Draw Stop zone
	recta.x = 300;
	recta.y = 300 - robot_clearence_width * 100;
	rectb.x = 300 + robot_stop_zone * 100;
	rectb.y = 300 + robot_clearence_width * 100;
	cvRectangle(raw_img,recta, rectb, CV_RGB(255,0,0),1,1,0);
	// draw turn zones
	recta.x = 300 + robot_stop_zone * 100;
	recta.y = 300 - (robot_clearence_width + robot_turn_zone_extra_width) * 100;
	rectb.x = recta.x + robot_turn_zone * 100;
	rectb.y = 300 - 1;
	cvRectangle(raw_img,recta, rectb, CV_RGB(255,0,255),1,1,0);
	recta.x = 300 + robot_stop_zone * 100;
	recta.y = 300 + (robot_clearence_width + robot_turn_zone_extra_width) * 100;
	rectb.x = recta.x + robot_turn_zone * 100;
	rectb.y = 300 + 1;
	cvRectangle(raw_img,recta, rectb, CV_RGB(0,255,0),1,1,0);
	// Draw boxes
	double box_height = row_box_height / row_box_count;
	for (int i = 0; i < row_box_count; i++){
		recta.x = 300 + (row_box_start_value + box_height*i) * 100;
		recta.y = 300 + (robot_clearence_width) * 100;
		rectb.x = recta.x + box_height * 100;
		rectb.y = 300 + (robot_clearence_width + row_box_width) * 100;
		//cvRectangle(raw_img,recta, rectb, CV_RGB(0,128,255),1,1,0);
	}

	// Clear the objects message
	object_msg.left_blocked = 0;
	object_msg.right_blocked = 0;
	object_msg.stop_zone_occupied = 0;
	// Check quadrants without OpenCV
	double r, alpha, x, y;
	for (int i = 0; i < size; i++) {
		x = cloud.points[i].x;
		y = cloud.points[i].y;
		if (x > 0 && x < robot_stop_zone){
			if (y > -robot_clearence_width && y < robot_clearence_width) {	// Full stop zone
				object_msg.stop_zone_occupied = 1;
				point1.x = ((int)(cloud.points[i].x * 100) + 300);
				point1.y = ((int)(cloud.points[i].y * 100) + 300);
				cvCircle(raw_img,point1,1,CV_RGB(255,0,0));	// Marks the point with a circle
			}
		}
		else if (x > robot_stop_zone && x < (robot_stop_zone + robot_turn_zone)){
			if (y > 0 && y < (robot_clearence_width + robot_turn_zone_extra_width)) {
				// If there are objects in the RIGHT quadrant
				r = sqrt(x * x + y * y);
				alpha = atan2(y,x);
				if (object_msg.right_blocked == 0){
					object_msg.right_blocked = 1;
					object_msg.closest_object_distance_right = r;
					object_msg.closest_object_angle_right = alpha;
				}else{
					if (r < object_msg.closest_object_distance_right){
						object_msg.closest_object_distance_right = r;
						object_msg.closest_object_angle_right = alpha;
					}
				}
				//ROS_INFO("RIGHT r: %f, ang: %f", r, alpha);
				point1.x = ((int)(cloud.points[i].x * 100) + 300);
				point1.y = ((int)(cloud.points[i].y * 100) + 300);
				cvCircle(raw_img,point1,1,CV_RGB(255,255,0));	// Marks the point with a circle
			}else if (y < 0 && y > -(robot_clearence_width + robot_turn_zone_extra_width)) {
				// If there are objects in the LEFT quadrant
				r = sqrt(x * x + y * y);
				alpha = atan2(y,x);
				if (object_msg.left_blocked == 0){
					object_msg.left_blocked = 1;
					object_msg.closest_object_distance_left = r;
					object_msg.closest_object_angle_left = alpha;
				}else{
					if (r < object_msg.closest_object_distance_left){
						object_msg.closest_object_distance_left = r;
						object_msg.closest_object_angle_left = alpha;
					}
				}
				//ROS_INFO("LEFT r: %f, ang: %f", r, alpha);
				point1.x = ((int)(cloud.points[i].x * 100) + 300);
				point1.y = ((int)(cloud.points[i].y * 100) + 300);
				cvCircle(raw_img,point1,1,CV_RGB(0,255,255));	// Marks the point with a circle
			}
		}
	}

	// Publish msg
	if (object_msg.right_blocked == 1 || object_msg.left_blocked == 1 || object_msg.stop_zone_occupied == 1){
		++object_msg.header.seq;
		object_msg.header.stamp = ros::Time::now();
		object_publisher.publish(object_msg);
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

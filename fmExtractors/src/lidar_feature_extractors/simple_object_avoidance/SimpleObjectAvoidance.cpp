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
	nh = ros::NodeHandle("~");
	// ROS PARAMETERS
	nh.param<string>("laser_scan_topic", laser_scan_topic, "laser_scan_topic");
	nh.param<int>("show_image", show_image, 1);
	nh.param<double>("robot_clearence_width", robot_clearence_width, 0.2);
	nh.param<double>("robot_stop_zone", robot_stop_zone, 0.2);
	nh.param<double>("robot_turn_zone", robot_turn_zone, 0.3);
	nh.param<double>("robot_turn_zone_extra_width", robot_turn_zone_extra_width, 0.05);
	nh.param<string>("object_topic", object_topic, "object_topic");
	nh.param<string>("object_row_topic", object_row_topic, "object_row_topic");
	nh.param<string>("wheel_speeds_topic", wheel_speeds_topic, "wheel_speeds_topic");

	// For row boxes
	nh.param<double>("row_box_start_value", row_box_start_value, -0.2);
	nh.param<double>("row_box_height", row_box_height, 1);
	nh.param<double>("row_box_width", row_box_width, 0.2);
	nh.param<int>("row_box_count", row_box_count, 5);

	// ROS PUBLISHERS AND SUBSCRIBERS
	laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 1, &SimpleObjectAvoidance::laserScanCallback, this);
	object_publisher = n.advertise<fmMsgs::detected_objects>(object_topic.c_str(), 1);
	object_row_publisher = n.advertise<fmMsgs::object_row>(object_row_topic.c_str(), 1);
	wheel_speeds_subscriber = n.subscribe<fmMsgs::float_data>(wheel_speeds_topic.c_str(), 1, &SimpleObjectAvoidance::wheelCallback, this);
	
	// Init messages
	object_row_msg.size = row_box_count;
	object_row_msg.resolution = row_box_height / row_box_count;
	object_row_msg.row_start_position = row_box_start_value;
	
	
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
	
	// Initial calculations
	for(int n = 0; n < row_box_count; n++) {
		object_row_msg.left_row.push_back(0);
		object_row_msg.right_row.push_back(0);
	}
	row_individual_box_height = row_box_height / row_box_count;
}

SimpleObjectAvoidance::~SimpleObjectAvoidance() {
	if (show_image){
		cvDestroyWindow(framedWindowName);
	}
}

void SimpleObjectAvoidance::wheelCallback(const fmMsgs::float_data::ConstPtr& speeds) {
	new_speeds = 1;
	wheel_speed_right = speeds->data[0];
	wheel_speed_left = speeds->data[1];
}


void SimpleObjectAvoidance::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan){
	//ROS_INFO("Laser Scan Callback");
	
	if(show_image) {
		// Clear image
		cvZero(raw_img);
	}
	// Create PointCloud from laser scan
	sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection projector;
	projector.projectLaser(*laser_scan, cloud);
	
	// Reset row vectors
	for(int n = 0; n < row_box_count; n++) {
		object_row_msg.left_row[n] = 0;
		object_row_msg.right_row[n] = 0;
	}
	
	int size = cloud.points.size();
	CvPoint point1;
	
	if(show_image) {
		// create img data from cloud data
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
		// Right side
		recta.x = 300 + (row_box_start_value + box_height*i) * 100;
		recta.y = 300 + (robot_clearence_width) * 100;
		rectb.x = recta.x + box_height * 100;
		rectb.y = 300 + (robot_clearence_width + row_box_width) * 100;
		cvRectangle(raw_img,recta, rectb, CV_RGB(0,128,255),1,1,0);
		
		// Left side
		recta.x = 300 +	 (row_box_start_value + box_height*i) * 100;
		recta.y = 300 - (robot_clearence_width) * 100;
		rectb.x = recta.x + box_height * 100;
		rectb.y = 300 - (robot_clearence_width + row_box_width) * 100;
		cvRectangle(raw_img,recta, rectb, CV_RGB(0,128,255),1,1,0);
		
		}
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
			if (y > -robot_clearence_width && y < robot_clearence_width) {
				// Full stop zone
				object_msg.stop_zone_occupied = 1;
				if(show_image) {
					point1.x = ((int)(cloud.points[i].x * 100) + 300);
					point1.y = ((int)(cloud.points[i].y * 100) + 300);
					cvCircle(raw_img,point1,1,CV_RGB(255,0,0));	// Marks the point with a circle
				}
			}
		} else if (x > robot_stop_zone && x < (robot_stop_zone + robot_turn_zone)){
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
				if(show_image) {
					point1.x = ((int)(cloud.points[i].x * 100) + 300);
					point1.y = ((int)(cloud.points[i].y * 100) + 300);
					cvCircle(raw_img,point1,1,CV_RGB(255,255,0));	// Marks the point with a circle
				}
			} else if (y < 0 && y > -(robot_clearence_width + robot_turn_zone_extra_width)) {
				// If there are objects in the LEFT quadrant
				r = sqrt(x * x + y * y);
				alpha = atan2(y,x);
				if (object_msg.left_blocked == 0){
					object_msg.left_blocked = 1;
					object_msg.closest_object_distance_left = r;
					object_msg.closest_object_angle_left = alpha;
				} else {
					if (r < object_msg.closest_object_distance_left){
						object_msg.closest_object_distance_left = r;
						object_msg.closest_object_angle_left = alpha;
					}
				}
				//ROS_INFO("LEFT r: %f, ang: %f", r, alpha);
				if(show_image) {
					point1.x = ((int)(cloud.points[i].x * 100) + 300);
					point1.y = ((int)(cloud.points[i].y * 100) + 300);
					cvCircle(raw_img,point1,1,CV_RGB(0,255,255));	// Marks the point with a circle
				}
			}
		}
		
		// Row detection
		if(x > row_box_start_value && x < row_box_start_value + row_box_height)
		{
			// Right row
			if(y > robot_clearence_width && y < robot_clearence_width + row_box_width) {
				for(int n = 0; n < row_box_count; n++)
				{
					if (x > (row_box_start_value + n * row_individual_box_height) && x < ((row_box_start_value + n * row_individual_box_height) + row_individual_box_height)) {
						object_row_msg.right_row[n]++;
					}
				}
				if(show_image) {
					// Add points
					point1.x = ((int)(cloud.points[i].x * 100) + 300);
					point1.y = ((int)(cloud.points[i].y * 100) + 300);
					cvCircle(raw_img,point1,1,CV_RGB(255,0,0));	// Marks the point with a circle
				}
			}
			
			// Left row
			if(y < -robot_clearence_width && y > -(robot_clearence_width + row_box_width)) {
				for(int n = 0; n < row_box_count; n++)
				{
					if (x > (row_box_start_value + n * row_individual_box_height) && x < ((row_box_start_value + n * row_individual_box_height) + row_individual_box_height)) {
						object_row_msg.left_row[n]++;
					}
				}
				if(show_image) {
					// Add points
					point1.x = ((int)(cloud.points[i].x * 100) + 300);
					point1.y = ((int)(cloud.points[i].y * 100) + 300);
					cvCircle(raw_img,point1,1,CV_RGB(255,255,0));	// Marks the point with a circle
				}
			}
		}
	}
	
	//ROS_INFO("Right row: %d %d %d %d %d %d %d %d %d %d", right_row[0], right_row[1], right_row[2], right_row[3], right_row[4], right_row[5], right_row[6], right_row[7], right_row[8], right_row[9]);
	//ROS_INFO("Left row: %d %d %d %d %d %d %d %d %d %d", left_row[0], left_row[1], left_row[2], left_row[3], left_row[4], left_row[5], left_row[6], left_row[7], left_row[8], left_row[9]);
	
	++object_row_msg.header.seq;
	object_row_msg.header.stamp = ros::Time::now();
	object_row_publisher.publish(object_row_msg);
	
	// Publish msg
	//if (object_msg.right_blocked == 1 || object_msg.left_blocked == 1 || object_msg.stop_zone_occupied == 1){
		++object_msg.header.seq;
		object_msg.header.stamp = ros::Time::now();
		object_publisher.publish(object_msg);
	//}
	if(show_image) {
		cvShowImage(framedWindowName,raw_img);
	}
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

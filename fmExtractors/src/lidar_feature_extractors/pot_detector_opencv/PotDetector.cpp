/*
 * PotDetector.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: Morten Ege Jensen
 */

#include "PotDetector.h"

using namespace std;
/*
 * Constructor
 */
PotDetector::PotDetector() {
	rawData_img = cvCreateImage(cvSize(600, 600), 8, 3);
	working_img = cvCreateImage(cvSize(600, 600), 8, 1);
	// For timing purposes
	avg_time = boost::circular_buffer<int>(avg_time_buffer_size);
	//	Initialize timing buffer
	for (int i = 0; i < avg_time.size(); i++)
	{
		avg_time.push_back(0);
	}
}
/*
 * Destructor
 */
PotDetector::~PotDetector() {

}

/*
 * Clears the raw_image
 */
void PotDetector::clearRawImg(){
	cvZero(rawData_img);
	storage = cvCreateMemStorage(0);
	lines = 0;
}

/*
 * Callback-function for the laser_scan. Runs Hough Lines calculation and chooses the appropriate lines
 * to represent rows. Specifies if they're on the left or right side and calculates the distance in
 * meterss from the center (robot laser position) to the line, and the angle between the x-axis and the line.
 */
void PotDetector::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan){
	// Test the time
	struct timeval start, end;
	long mtime, seconds, useconds;
	static int min_time = 99, max_time = 0;
	gettimeofday(&start, NULL);

	// Clear the raw image data
	clearRawImg();

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
			cvCircle(rawData_img,point1,1,CV_RGB(255,255,255));	// Marks the point with a circle
		}
	}
	// Transform to gray scale image for hough transform
	cvCvtColor(rawData_img, working_img, CV_BGR2GRAY);
	//cvGaussianBlur( working_img, working_img, Size(9, 9), 2, 2 );// Doesn't exist like this

//	cvDilate(rawData_img,rawData_img,NULL,6);	// Works better with lines
//	cvErode(rawData_img,rawData_img,NULL,4);	// works better with lines

	// Loads of calculations to find the lines
	double rho = 0, theta = 0, a, b, x0, y0;
	// For avg calculation
	double rrhomean = 0, lrhomean = 0, rthetamean = 0, lthetamean = 0, rmeancount = 0, lmeancount = 0;
	// For final calculation
	double dist= 0, ang = 0, ldist = 0, lang = 0, rdist = 0, rang = 0;

	// Detect lines using Hough Lines algorithm
	CvPoint pointa, pointb, pointc, pointd, point_center, isection;
	lines = cvHoughLines2(working_img, storage, CV_HOUGH_STANDARD, 1, CV_PI/ 180, 60, 0, 0);
	for (int i = 0; i < MIN(lines->total, 20); i++){
		// calculate line
		float* line = (float*) cvGetSeqElem(lines, i);
		rho = line[0];
		theta = line[1];

		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		pointa.x = cvRound(x0 + 600 * (-b));
		pointa.y = cvRound(y0 + 600 * (a));
		pointb.x = cvRound(x0 - 600 * (-b));
		pointb.y = cvRound(y0 - 600 * (a));

		// Calculate normal through center-point
		point_center.x = 300;
		point_center.y = 300;
		pointc.x = point_center.x + (pointb.y - pointa.y);
		pointc.y = point_center.y - (pointb.x - pointa.x);
		pointd.x = point_center.x - (pointb.y - pointa.y);
		pointd.y = point_center.y + (pointb.x - pointa.x);

		// Calculate intersection between line and its normal
		isection = intersection(pointc, pointd, pointa, pointb);
		// if distance is OK, its a navigable row
		double dist = distance(point_center, isection);
		if ((dist / 100) < max_dist_to_rows){	// Now its 0.6 meters
			if (isection.y < 300){	// This will be a row on the left side of the robot
				lrhomean += rho;
				lthetamean += theta;
				lmeancount++;
				//cvLine(rawData_img, pointa, pointb, CV_RGB(0,255,0), 1, CV_AA, 0);
			}else{					// this will be the row on the right side of the robot
				rrhomean += rho;
				rthetamean += theta;
				rmeancount++;
				//cvLine(rawData_img, pointa, pointb, CV_RGB(255,0,0), 1, CV_AA, 0);
			}
			//cvLine(rawData_img, pointc, pointd, CV_RGB(255,255,0), 1, CV_AA, 0);	// Draws the normal
			//cvCircle(rawData_img,isection,4,CV_RGB(0,255,255));						// Draws the intersecting point
		}
	}
	/////////////////////////////////////////////////////////////
	// Find the average lines, calculate the distance and angle
	// Left row
	if (lmeancount){		// If there is a left row
		rho = lrhomean / lmeancount;
		theta = lthetamean / lmeancount;
		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		pointa.x = cvRound(x0 + 600 * (-b));
		pointa.y = cvRound(y0 + 600 * (a));
		pointb.x = cvRound(x0 - 600 * (-b));
		pointb.y = cvRound(y0 - 600 * (a));
		// Calculate normal through center-point
		point_center.x = 300;
		point_center.y = 300;
		pointc.x = point_center.x + (pointb.y - pointa.y);
		pointc.y = point_center.y - (pointb.x - pointa.x);
		pointd.x = point_center.x - (pointb.y - pointa.y);
		pointd.y = point_center.y + (pointb.x - pointa.x);

		// Calculate intersection between line and its normal
		isection = intersection(pointc, pointd, pointa, pointb);
		ldist = distance(point_center, isection) / 100;
		lang = CV_PI/2 - theta;
		// Draw the line representing the row
		cvLine(rawData_img, pointa, pointb, CV_RGB(0,255,0), 1, CV_AA, 0);
	}

	// Right row
	if (rmeancount){		// If there is a right row
		rho = rrhomean / rmeancount;
		theta = rthetamean / rmeancount;
		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		pointa.x = cvRound(x0 + 600 * (-b));
		pointa.y = cvRound(y0 + 600 * (a));
		pointb.x = cvRound(x0 - 600 * (-b));
		pointb.y = cvRound(y0 - 600 * (a));
		// Calculate normal through center-point
		point_center.x = 300;
		point_center.y = 300;
		pointc.x = point_center.x + (pointb.y - pointa.y);
		pointc.y = point_center.y - (pointb.x - pointa.x);
		pointd.x = point_center.x - (pointb.y - pointa.y);
		pointd.y = point_center.y + (pointb.x - pointa.x);

		// Calculate intersection between line and its normal
		isection = intersection(pointc, pointd, pointa, pointb);
		rdist = distance(point_center, isection) / 100;
		rang = CV_PI/2 - theta;
		// Draw the line representing the row
		cvLine(rawData_img, pointa, pointb, CV_RGB(255,0,0), 1, CV_AA, 0);
	}
	/////////////////// end of final calculation /////////////////////////

	// Do something with the angles and distances
	row.header.stamp = ros::Time::now();
	++row.header.seq;
	row.rightvalid = 0;
	row.leftvalid = 0;
	
	if (lmeancount){
		//ROS_INFO("Only left! dist: %f, ang %f", ldist, lang);
		row.leftvalid = 1;
		row.leftdistance = ldist;
		row.leftangle = lang;
	}
	
	if(rmeancount){
		//ROS_INFO("Only right! dist: %f, ang %f", rdist, rang);
		row.rightvalid = 1;
		row.rightdistance = rdist;
		row.rightangle = rang;
	}

	if (rmeancount || lmeancount){	// If rows are detected, publish message
		row_pub.publish(row);
	}
	
	// Draw vertical line
	CvPoint p_vertical_a, p_vertical_b;
	p_vertical_a.x = 300, p_vertical_a.y = 0;
	p_vertical_b.x = 300, p_vertical_b.y = 600;
	cvLine(rawData_img, p_vertical_a, p_vertical_b, CV_RGB(0,0,255), 1, CV_AA, 0);

	if (show_image_boolean){
		// Show the image
		cvShowImage(framedWindowName, rawData_img);
	}

	// Test the TIME
	gettimeofday(&end, NULL);

	seconds  = end.tv_sec  - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

	if (mtime < min_time){
		min_time = mtime;
	}
	if (mtime > max_time || mtime > 30){
		max_time = mtime;
	}
	// average time
	avg_time.push_back(mtime);
	int sum = 0;
	for (int i = 0; i < avg_time.size(); i++){
		sum += avg_time[i];
	}
	sum = sum / avg_time.size();

	ROS_INFO("time: %ld, min: %ld, max: %ld, avg: %d", mtime, min_time, max_time, sum);
	//printf("Elapsed time: %ld milliseconds\n", mtime);

}

/*
 * Calculates the distance between two points
 */
double PotDetector::distance(CvPoint p1, CvPoint p2){
	int dx = p1.x - p2.x;
	int dy = p1.y - p2.y;
	return sqrt(dx*dx + dy*dy);
}

/*
 * Calculates the intersecting point between two lines spanned (p1, p2) and (p3, p4)
 * Credit to Søren Hundevadt Nielsen.
 */
CvPoint PotDetector::intersection(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4){
	CvPoint intersection;
	// Store the values for fast access and easy
	// equations-to-code conversion
	float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0)
		return intersection;

	// Get the x and y
	float pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
	float x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	float y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < MIN(x2, x1) || x > MAX(x2, x1) || x < MIN(x4, x3) || x
			> MAX(x4, x3))
		return intersection;
	if (y < MIN(y2, y1) || y > MAX(y2, y1) || y < MIN(y4, y3) || y
			> MAX(y4, y3))
		return intersection;

	intersection.x = x;
	intersection.y = y;

	return intersection;
}

/*
 * Main loop
 * Initializes ros, its parameters and the subscribers/publishers
 * Instatiates an object to run the laser_scan callback
 * Creates a handle to OpenCV window frames, if this is specified in the parameters.
 */
int main(int argc, char** argv){
	// Initialize ROS
	ros::init(argc, argv, "pot_detector");
	// Ros nodehandles
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
	// Instatiate object
	PotDetector pd;
	// Ros params
	string laser_scan_topic, row_topic;
	nh.param<string>("laser_scan_topic", laser_scan_topic, "laser_scan_topic");
	nh.param<string>("row_topic", row_topic, "row_topic");
	nh.param<double>("max_dist_to_rows", pd.max_dist_to_rows, 0.6);
	nh.param<int>("show_image", pd.show_image_boolean, 1);
	nh.param<int>("avg_time_buffer_size", pd.avg_time_buffer_size, 10);
	// Subscribes and publishers
	ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 10, &PotDetector::laserScanCallback, &pd);
	pd.row_pub = n.advertise<fmMsgs::row>(row_topic.c_str(), 10);
	// Start a thread for windows if it is TRUE
	if (pd.show_image_boolean){
		// Create a window
		pd.framedWindowName = "OpenCVWindowName";
		cvNamedWindow(pd.framedWindowName);
		cvStartWindowThread();
		// Show image test
		//cvShowImage(framedWindowName, rawData_img);
	}

	// Spin
	ros::spin();
	// Close the window if it is TRUE
	if (pd.show_image_boolean){
		cvDestroyWindow(pd.framedWindowName);
	}
	return 0;
}

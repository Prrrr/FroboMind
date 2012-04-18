/*
 * PotDetector.h
 *
 *  Created on: Apr 13, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef POTDETECTOR_H_
#define POTDETECTOR_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "fmMsgs/row.h"
#include "laser_geometry/laser_geometry.h"
// OpenCv includes
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
// For timing puposes
#include "sys/time.h"
#include "boost/circular_buffer.hpp"
using namespace std;
class PotDetector {
private:
	// OpenCv
	IplImage* rawData_img;
	IplImage* working_img;
	CvMemStorage* storage;
	CvSeq* lines;

	// Functions
	void clearRawImg();
	CvPoint intersection(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4);
	double distance(CvPoint p1, CvPoint p2);

	// For timing puposes
	boost::circular_buffer<int> avg_time;
public:
	PotDetector();
	virtual ~PotDetector();
	// OpenCV window
	const char* framedWindowName;
	// parameters
	double max_dist_to_rows;
	int show_image_boolean, avg_time_buffer_size;
	//Callbacks
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	
	ros::Publisher row_pub;
	fmMsgs::row row;
};

#endif /* POTDETECTOR_H_ */

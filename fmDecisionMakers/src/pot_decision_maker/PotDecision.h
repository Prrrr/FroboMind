/*
 * PotDecision.h
 *
 *  Created on: Apr 16, 2012
 *      Author: Nis Sarup
 */

#ifndef POTDECISION_H_
#define POTDECISION_H_
#include "ros/ros.h"
#include "fmMsgs/row.h"
#include "fmMsgs/float_data.h"
#include "fmMsgs/gyroscope.h"
#include "math.h"
using namespace std;
class PotDecision {
private:
	// New Data flags
	int new_speeds, new_gyro, new_l_row, new_r_row;
	
	double wheel_speed_right, wheel_speed_left;
	double gyro_z;
	
	// Row data
	double rightdistance, rightangle;
	double leftdistance, leftangle;
	
public:
	PotDecision();
	virtual ~PotDecision();
	
	// Functions
	void rowCallback(const fmMsgs::row::ConstPtr& row);
	void wheelCallback(const fmMsgs::float_data::ConstPtr& speeds);
	void gyroCallback(const fmMsgs::gyroscope::ConstPtr& gyro);
	void timerCallback(const ros::TimerEvent& event);
	void calculate_odometry();
};

#endif /* POTDECISION_H_ */

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
#include "fmMsgs/detected_objects.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"
#include "pid.h"
	 
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
	
	// Navigation data
	double cross_track_error;
	
	// Object data
	int new_object_message_received, new_left_object, new_right_object, new_stop;
	double object_left_distance, object_right_distance, object_left_angle, object_right_angle;
	
	// msgs
	geometry_msgs::TwistStamped twist_msg;
public:
	PotDecision();
	virtual ~PotDecision();
	
	// Parameters
	double linear_mean_velocity, mean_driving_distance_from_rows;
	double cte_weight_angle, cte_weight_distance;
	double base_link_radius_to_wheels;
	double time_s;
	double cte_kp, cte_ki, cte_kd;

	// Functions
	void rowCallback(const fmMsgs::row::ConstPtr& row);
	void wheelCallback(const fmMsgs::float_data::ConstPtr& speeds);
	void gyroCallback(const fmMsgs::gyroscope::ConstPtr& gyro);
	void objectCallback(const fmMsgs::detected_objects::ConstPtr& objects);
	void timerCallback(const ros::TimerEvent& event);
	void calculate_odometry();
	void calculate_twist();

	// Publisher
	ros::Publisher twist_pub;
	
	// PID
	PID cte_pid; 
};

#endif /* POTDECISION_H_ */
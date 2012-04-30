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
#include "fmMsgs/hilde_states.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"
#include "pid.h"
#include "fmMsgs/object_row.h"
#include "vector"
	
	 
using namespace std;
class PotDecision {
private:
	// New Data flags
	// Remember to initialize to zero in the constructor
	int new_speeds, new_gyro, new_l_row, new_r_row, new_object_row, new_object_message_received;
	
	double wheel_speed_right, wheel_speed_left;
	double gyro_z;
	
	// Row data
	double rightdistance, rightangle;
	double leftdistance, leftangle;
	
	// Navigation data
	double cross_track_error;
	
	// Object data
	int new_left_object, new_right_object, new_stop;
	double object_left_distance, object_right_distance, object_left_angle, object_right_angle;
	
	// msgs
	geometry_msgs::TwistStamped twist_msg;
	fmMsgs::hilde_states row_state_msg;
	
	// Object Row Data
	vector<int> object_row_left;
	vector<int> object_row_right;
	double object_row_resolution;
	double object_row_start_position;
	double object_row_end_position_left;
	double object_row_end_position_right;
	double object_row_fill_percent_left;
	double object_row_fill_percent_right;
	ros::Time object_row_data_last_update;
	int row_state;
	
public:
	PotDecision();
	virtual ~PotDecision();
	
	// Parameters
	double linear_mean_velocity, mean_driving_distance_from_rows;
	double cte_weight_angle, cte_weight_distance;
	double base_link_radius_to_wheels;
	double time_s;
	double cte_kp, cte_ki, cte_kd;
	int object_row_box_filled_threshold;
	double object_row_threshold;
	double dead_reckoning_turn_rate;

	// Functions
	void rowCallback(const fmMsgs::row::ConstPtr& row);
	void objectRowCallback(const fmMsgs::object_row::ConstPtr& row);
	void wheelCallback(const fmMsgs::float_data::ConstPtr& speeds);
	void gyroCallback(const fmMsgs::gyroscope::ConstPtr& gyro);
	void objectCallback(const fmMsgs::detected_objects::ConstPtr& objects);
	void timerCallback(const ros::TimerEvent& event);
	void calculate_odometry();
	void calculate_twist_from_object_boxes();
	void extract_object_row_data();
	void run_state_machine();

	// Publisher
	ros::Publisher twist_pub;
	ros::Publisher row_state_pub;
	
	// PID
	PID cte_pid; 
};

#endif /* POTDECISION_H_ */

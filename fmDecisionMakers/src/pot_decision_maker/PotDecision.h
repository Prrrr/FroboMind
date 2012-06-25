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


struct state_space{
	double b;
	double x, y, th;
	void set_zero(){
		x = 0;
		y = 0;
		th = 0;
	}
	void calc_odom(double left, double right, double dt){
		double V = (left + right)/2;
		double w = (right - left)/(2*b);
		x = x + V*cos(th);
		y = y + V*sin(th);
		th = th + w*dt;
	}
}state_space;

struct row_hole_finder{
	int hole_start, hole_end, row_start, row_end;
	vector<bool> list;
	void clear_values(){
		list.clear();
		hole_start = -1;
		hole_end = -1;
		row_end = -1;
		row_start = -1;
	}

	void run_scheme(){
		// Hole detecting scheme
		for (int i = 0; i < list.size(); i++){
			if (i == 0){
				if (list[i]){
					row_start = i;
				}else{
					hole_start = i;
				}
			}else{
				if (list[i] && !list[i-1]){
					if (row_start == -1){
						row_start = i;
					}
					hole_end = i - 1;
				}else if (!list[i] && list[i-1]){
					if (hole_start == -1){
						hole_start = i;
					}
					row_end = i - 1;
				}
			}
		}
		if (hole_start > -1 && hole_end == -1){
			hole_end = list.size();
		}
		if (row_start > -1 && row_end == -1){
			row_end = list.size();
		}
	}
};

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
	int next_turn_direction;
	
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
	row_hole_finder left_row_finder, right_row_finder;
	
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

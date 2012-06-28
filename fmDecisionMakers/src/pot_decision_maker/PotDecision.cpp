/*
 * PotDecision.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: Nis Sarup
 */


#include "PotDecision.h"
#include <vector>
using namespace std;

// Statemachine defines
	enum { 
		STM_START, 
		STM_DETECT_ROW, 
		STM_DRIVE, 
		STM_STOP,
		STM_TURNING,
		STM_END_OF_ROW,
		STM_EXIT_ROW,
		STM_HEADLAND,
		STM_HEADLAND_ROW,
		STM_HEADLAND_HOLE,
		STM_MARKER
	};
 
 	enum { 
		RST_BETWEEN_ROWS,
		RST_RIGHT_ROW,
		RST_LEFT_ROW,
		RST_NO_ROW
	 };
	 
	 enum {
		 RIGHT,
		 LEFT
	 };
	int test [4] = {2,2,2,2};
	vector<int> prevpath;
	vector<int> rempath(test,test + sizeof(test) / sizeof(int));
	int pattern = 1;
/*
 * Constructor
 */
PotDecision::PotDecision() {
	new_speeds = 0;
	new_gyro = 0;
	new_l_row = 0;
	new_r_row = 0;
	new_object_row = 0;
	new_object_message_received = 0;
	saved_wii_state = 0;
	
	// Object row data initialization
	object_row_resolution = 0;
	object_row_start_position = 0;
	object_row_data_last_update = ros::Time(0);
	object_row_end_position_left = 1000;
    object_row_end_position_right = 1000;
	row_state = RST_NO_ROW;
	
	// Navigation
	if (pattern == 0 ) {
	next_turn_direction = RIGHT;
	}
	else if (rempath[0] == 1) {
	next_turn_direction = RIGHT;
	}
	else if (rempath[0] == 2) {
	next_turn_direction = LEFT;
	}
	// state space
	state_space.b = 0.18;
}

/*
 * Destructor
 */
PotDecision::~PotDecision() {

}

void PotDecision::run_state_machine() {
	static int state = STM_START;
	int publish_twist = 0;
	static int between_row_counter = 0, right_row_counter = 0, left_row_counter = 0;
//	static int passed_row = 0;
//	static int passed_hole = 0;
	static int rowcount = 0;
	static int holecount = 0;	
	double speed_factor = 0;
	double gyro_offset = 0;
	double gyro = 0;
	
	// Build twist
	++twist_msg.header.seq;
	twist_msg.header.stamp = ros::Time::now();
	twist_msg.twist.linear.x = 0;
	twist_msg.twist.angular.z = 0;
	
	//set odom min.
	double odom_min = 0;
	
	switch (state) {
		case STM_START:
			if(row_state != RST_NO_ROW)
			{
				state = STM_DRIVE;
				ROS_WARN("State: Drive");
				between_row_counter = 0;
				right_row_counter = 0;
				left_row_counter = 0;
				gyro_offset = gyro_z;
				
			}
			break;
			
		case STM_DRIVE:
			// Driving. Calculate twist when new box-info comes in
			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}
			if(rempath.size() == 0)
			{
			}
			else if (rempath[0] == 1) {
			next_turn_direction = RIGHT;
			}
			else if (rempath[0] == 2) {
			next_turn_direction = LEFT;
			}

			// Increment counters
			if(row_state == RST_BETWEEN_ROWS) {
				between_row_counter++;
				speed_factor = (object_row_fill_percent_left + object_row_fill_percent_right) / 2;
			} else if(row_state == RST_RIGHT_ROW) {
				right_row_counter++;
				speed_factor = object_row_fill_percent_right;
			} else if(row_state == RST_LEFT_ROW) {
				left_row_counter++;
				speed_factor = object_row_fill_percent_left;
			}
			
			// Clamp speed factor
			if(speed_factor > 1)
			{
				speed_factor = 1;
			} else if (speed_factor < 0.5) {
				speed_factor = 0.5;
			}
			
			twist_msg.twist.linear.x = speed_factor * linear_mean_velocity;
			
			// Detect ending row
			if(right_row_counter > left_row_counter && right_row_counter > between_row_counter) {
				twist_msg.twist.angular.z += 0.2;
				if(object_row_end_position_right <= 0) {
					if (pattern == 0){
					next_turn_direction = RIGHT;
					}
					ROS_INFO("Going right");
					//state = STM_TURNING;
					// clear state space values
					state_space.set_zero();
					state = STM_EXIT_ROW;
					ROS_WARN("State: EXIT_ROW");
				}
			} else if (left_row_counter > right_row_counter && left_row_counter > between_row_counter) {
				twist_msg.twist.angular.z -= 0.2;
				if(object_row_end_position_left <= 0) {
					if (pattern == 0) {
					next_turn_direction = LEFT;
					}
					ROS_INFO("Going left");
					//state = STM_TURNING;
					// clear state space values
					state_space.set_zero();
					state = STM_EXIT_ROW;
					ROS_WARN("State: Exit row");
				}
			} else {
				if (next_turn_direction == RIGHT) {
					if(object_row_end_position_right <= 0) {
						ROS_INFO("Going right");
						//state = STM_TURNING;
						// clear state space values
						state_space.set_zero();
						state = STM_EXIT_ROW;
						ROS_WARN("State: Exit row");
					}
				} else {
					if(object_row_end_position_left <= 0) {
						ROS_INFO("Going left");
						//state = STM_TURNING;
						// clear state space values
						state_space.set_zero();
						state = STM_EXIT_ROW;
						ROS_WARN("State: Exit row");
					}
				}
			}
			twist_msg.twist.angular.z = cte_pid.run(twist_msg.twist.angular.z, 0.02);
			break;
		case STM_EXIT_ROW:
			// keep driving
			// To avoid crashing into ponies.
			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}

			twist_msg.twist.angular.z = abs(twist_msg.twist.angular.z);
			twist_msg.twist.angular.z += dead_reckoning_turn_rate;
			twist_msg.twist.linear.x = 0.5;

			// FLip the sign if we need to go left
			if(next_turn_direction == LEFT)
			{
				twist_msg.twist.angular.z = -twist_msg.twist.angular.z;
			}
			// keep calculating odometry
			gyro = gyro_z - gyro_offset;
			state_space.calc_odom(wheel_speed_left, wheel_speed_right, 0.02, gyro);
			ROS_INFO("state x:%f, y:%f, th:%f, gy_th:%f", state_space.x, state_space.y, state_space.th, state_space.gyro_th);
			if (abs(state_space.th) > M_PI/2){
				state = STM_HEADLAND;
				//passed_row = 0;
				rowcount = 0;
				holecount = 0;
				//passed_hole = 0;
				state_space.set_zero();
				ROS_WARN("State: headland");
				between_row_counter = 0;
				right_row_counter = 0;
				left_row_counter = 0;
			}
			break;
		case STM_HEADLAND:
			// drive straight
			// Driving. Calculate twist when new box-info comes in
			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}

			// Clamp speed factor
			speed_factor = 0.5;

			twist_msg.twist.linear.x = speed_factor * linear_mean_velocity;
			// check for a hole to enter

			if (next_turn_direction == LEFT){
			
				if (left_row_finder.list[1])
				{
				state = STM_HEADLAND_ROW;
				state_space.set_zero();
				rowcount++;				
				ROS_INFO("Entered HEADLAND_ROW state %d", rowcount);
				}							
				else if (!left_row_finder.list[1])
				{
				state = STM_HEADLAND_HOLE;
				state_space.set_zero();
				holecount++;
				rowcount++;
				ROS_INFO("Entered HEADLAND_HOLE state %d", holecount);
				}			
			}
			else if (next_turn_direction == RIGHT){
			
				if (right_row_finder.list[1])
				{
				rowcount++;
				ROS_INFO("Entered HEADLAND_ROW state %d", rowcount);				
				state = STM_HEADLAND_ROW;
				state_space.set_zero();
				}							
				else if (!right_row_finder.list[1])
				{
				holecount++;
				rowcount++;
				ROS_INFO("Entered HEADLAND_HOLE state %d", holecount);
				state = STM_HEADLAND_HOLE;
				state_space.set_zero();
				}			
			}
			/*if (next_turn_direction == LEFT){
				//keep calculating odometry
				state_space.calc_odom(wheel_speed_left, wheel_speed_right, 0.02);

				if (left_row_finder.hole_start <= 1 && left_row_finder.hole_end > 2){
					if(passed_hole == 1 && passed_row == 2) 
					{
					ROS_WARN("State: TURNING");
					state = STM_TURNING;
					}
					else if(passed_hole == passed_row-1 && state_space.x > 0.3)
					{
					state_space.set_zero();					
					passed_hole++;
					ROS_INFO("detected holes: %d",passed_hole);
					}

				}
				else if(left_row_finder.row_start <= 1 && left_row_finder.row_end > 2 && passed_hole == passed_row && state_space.x > 0.5  )
				{
				state_space.set_zero();
				passed_row++;
				ROS_INFO("detected rows: %d",passed_row);
				}
			}else{	
				//keep calculating odometry
				state_space.calc_odom(wheel_speed_left, wheel_speed_right, 0.02);
				
				if (right_row_finder.hole_start <= 1 && right_row_finder.hole_end > 2){
					if(passed_hole == 1 && passed_row == 2) 
					{
					ROS_WARN("State: TURNING");
					state = STM_TURNING;
					}
					else if(passed_hole == passed_row-1 && state_space.x > 0.3)
					{
					state_space.set_zero();
					passed_hole++;
					ROS_INFO("detected holes: %d",passed_hole);
					}
				}
				else if(right_row_finder.row_start <= 1 && right_row_finder.row_end > 2 && passed_hole == passed_row && state_space.x > 0.5 )
				{
				state_space.set_zero();
				passed_row++;
				ROS_INFO("detected rows: %d",passed_row);
				}
			}*/
			break;

		case STM_HEADLAND_ROW:

			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}

			// Clamp speed factor
			speed_factor = 0.5;

			twist_msg.twist.linear.x = speed_factor * linear_mean_velocity;
			
			//keep calculating odometry
			gyro = gyro_z - gyro_offset;
			state_space.calc_odom(wheel_speed_left, wheel_speed_right, 0.02, gyro);
			
			if (pattern == 0)
			{
			odom_min = 0.1;
			}
			else if (rempath[1] == rowcount) {
			odom_min = 0.1;			
			}		
			else {
			odom_min = 0.3;			
			}
			if (next_turn_direction == LEFT){
				if(!left_row_finder.list[1] && state_space.x > odom_min )	{
				state = STM_HEADLAND_HOLE;
				holecount++;
				state_space.set_zero();
				ROS_INFO("Entered HEADLAND_HOLE state %d", holecount);
				}
			}
			else if (next_turn_direction == RIGHT){
				if(!right_row_finder.list[1] && state_space.x > odom_min )	{
				state = STM_HEADLAND_HOLE;
				holecount++;
				state_space.set_zero();
				ROS_INFO("Entered HEADLAND_HOLE state %d", holecount);
				}
			}
			break;
		case STM_HEADLAND_HOLE:
			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}

			// Clamp speed factor
			speed_factor = 0.5;

			twist_msg.twist.linear.x = speed_factor * linear_mean_velocity;			
			
			//keep calculating odometry
			gyro = gyro_z - gyro_offset;
			state_space.calc_odom(wheel_speed_left, wheel_speed_right, 0.02, gyro);
	
			//if (holecount == 2 && rowcount == 2) {
			//ROS_WARN("State: TURNING");
			//state = STM_TURNING;
			//}
			if (pattern == 0){
				ROS_WARN("State: TURNING");
				state = STM_TURNING;
			}
			else if(holecount == rempath[1] && rowcount == rempath[1]) {
			ROS_WARN("State: TURNING");
			state = STM_TURNING;
			prevpath.push_back(rempath[0]);
			rempath.erase(rempath.begin());
			prevpath.push_back(rempath[0]);
			rempath.erase(rempath.begin());
			
			}
			
			else if (next_turn_direction == LEFT){
				if(left_row_finder.list[1] && state_space.x > 0.5 )	{
				state = STM_HEADLAND_ROW;
				rowcount++;
				state_space.set_zero();
				ROS_INFO("Entered HEADLAND_ROW state %d", rowcount);
				}
			}
			else if (next_turn_direction == RIGHT){
				if(right_row_finder.list[1] && state_space.x > 0.5 )	{
				state = STM_HEADLAND_ROW;
				rowcount++;
				state_space.set_zero();
				ROS_INFO("Entered HEADLAND_ROW state %d", rowcount);
				}
			}
			break;

		case STM_TURNING:
			
			// To avoid crashing into ponies.
			if(new_object_message_received)
			{
				new_object_message_received = 0;
				calculate_twist_from_object_boxes();
				publish_twist = 1;
			}
			
			twist_msg.twist.angular.z = abs(twist_msg.twist.angular.z);
			twist_msg.twist.angular.z += dead_reckoning_turn_rate;
			twist_msg.twist.linear.x = 0.5;
			
			// FLip the sign if we need to go left
			if(next_turn_direction == LEFT)
			{
				twist_msg.twist.angular.z = -twist_msg.twist.angular.z;
			}
			
			if(row_state != RST_NO_ROW)
			{
				if (rempath.size() == 0) {
				pattern = 0;
				// Turn the other way next time
				next_turn_direction = (next_turn_direction == RIGHT) ? LEFT : RIGHT;
				}
				else if (rempath[0] == 1) {
				next_turn_direction = RIGHT;
				}
				else {
				next_turn_direction = LEFT;
				}
				state = STM_DRIVE;
				ROS_WARN("State: Drive");
				between_row_counter = 0;
				right_row_counter = 0;
				left_row_counter = 0;
			}
			break;
		
		case STM_END_OF_ROW:
			publish_twist = 1;
			state = STM_STOP;
			ROS_WARN("State: Stop");
			break;
		
		case STM_STOP:
			break;
		case STM_MARKER:
			ROS_ERROR("Mark Found");
			state = STM_DRIVE;
			ROS_WARN("State: Drive");
			break;
			
		default:
			ROS_WARN("State: Default");
			break;
	}
	
	if (new_stop) {
		twist_msg.twist.linear.x = 0;
		twist_msg.twist.angular.z = 0;
		publish_twist = 1;
		new_stop = 0;
	}
	
	
	if(publish_twist)
	{
		twist_msg.twist.angular.z = cte_pid.run(twist_msg.twist.angular.z, 0.02);
		
		if (saved_wii_state == 0) {
			twist_msg.twist.linear.x = 0;
			twist_msg.twist.angular.z = 0;
		}
		
		twist_pub.publish(twist_msg);
		publish_twist = 0;
	}
	
	if (marker_found == 1){
		//marker_found = 0; / Only find marker ONCE
		state = STM_MARKER;
	}

	// Publish state
	++row_state_msg.header.seq;
	row_state_msg.header.stamp = ros::Time::now();
	row_state_msg.row_state = row_state;
	row_state_msg.state = state;
	row_state_msg.next_turn_direction = next_turn_direction;
	row_state_pub.publish(row_state_msg);
}


void PotDecision::extract_object_row_data() {
	int l_end_found = -1, r_end_found = -1;
	double fill_rate_left = 0, fill_rate_right = 0;
	int left_hole_width = 0, right_hole_width = 0, left_hole_valid = 0, right_hole_valid = 0;
	// Clear hole/row finder data
	left_row_finder.clear_values();
	right_row_finder.clear_values();
	for(int i = 0; i < object_row_left.size(); i++)
	{
		if(object_row_left[i] > object_row_box_filled_threshold) {
			l_end_found = i;
			fill_rate_left++;
			if(left_hole_width > 0)
			{
				left_hole_valid = 1;
			}
			left_row_finder.list.push_back(true);
		} else {
			left_row_finder.list.push_back(false);
			left_hole_valid = 0;
			left_hole_width++;
		}
		if(object_row_right[i] > object_row_box_filled_threshold) {
			r_end_found = i;
			fill_rate_right++;
			if(right_hole_width > 0)
			{
				right_hole_valid = 1;
			}
			right_row_finder.list.push_back(true);
		} else {
			right_row_finder.list.push_back(false);
			right_hole_valid = 0;
			right_hole_width++;
		}
	}

	right_row_finder.run_scheme();
//	ROS_INFO("RIGHT ROW: rs:%d re:%d hs:%d he:%d", right_row_finder.row_start, right_row_finder.row_end, right_row_finder.hole_start, right_row_finder.hole_end);
	left_row_finder.run_scheme();
//	ROS_INFO("LEFT ROW: rs:%d re:%d hs:%d he:%d", left_row_finder.row_start, left_row_finder.row_end, left_row_finder.hole_start, left_row_finder.hole_end);
	
	object_row_end_position_left = object_row_start_position + ((l_end_found + 1) * object_row_resolution);
	object_row_end_position_right = object_row_start_position + ((r_end_found + 1) * object_row_resolution);
	
	object_row_fill_percent_left = fill_rate_left / object_row_left.size();
	object_row_fill_percent_right = fill_rate_right / object_row_right.size();
	
	if(object_row_fill_percent_left > object_row_threshold && object_row_fill_percent_right > object_row_threshold && !left_hole_valid && !right_hole_valid)
	{
		// Between rows
		row_state = RST_BETWEEN_ROWS;
	} else if(object_row_fill_percent_left > object_row_threshold && !left_hole_valid)
	{
		// Row to the left
		row_state = RST_LEFT_ROW;
	} else if(object_row_fill_percent_right > object_row_threshold && !right_hole_valid)
	{
		// Row to the left
		row_state = RST_RIGHT_ROW;
	} else {
		// No rows! Panic!
		row_state = RST_NO_ROW;
	}
	
	object_row_data_last_update = ros::Time::now();
	
	//ROS_INFO("Right fill rate: %f", object_row_fill_percent_right);
	//ROS_INFO("Left fill rate: %f", object_row_fill_percent_left);
}

void PotDecision::objectRowCallback(const fmMsgs::object_row::ConstPtr& row) {
	//ROS_INFO("Object row Callback!");
	
	// Initialize the data and dataholders
	if(object_row_resolution == 0 && object_row_start_position == 0)
	{
		object_row_resolution = row->resolution;
		object_row_start_position = row->row_start_position;
		for(int n = 0; n < row->size; n++) {
			object_row_left.push_back(0);
			object_row_right.push_back(0);
		}
	}
	
	for(int n = 0; n < row->size; n++) {
		object_row_left[n]	= row->left_row[n];
		object_row_right[n]	= row->right_row[n];
	}
	
	new_object_row = 1;
}

void PotDecision::rowCallback(const fmMsgs::row::ConstPtr& row) {
	//ROS_INFO("Callback like a baws!!1!");
	new_l_row = row->leftvalid;
	new_r_row = row->rightvalid;
	rightdistance = row->rightdistance;
	rightangle = row->rightangle;
	leftdistance = row->leftdistance;
	leftangle = row->leftangle;
}

void PotDecision::wheelCallback(const fmMsgs::float_data::ConstPtr& speeds) {
	//ROS_INFO("Speeds right/left: %f, %f", speeds->data[1], speeds->data[0]);
	new_speeds = 1;
	wheel_speed_right = speeds->data[0];
	wheel_speed_left = speeds->data[1];
}

void PotDecision::objectCallback(const fmMsgs::detected_objects::ConstPtr& objects) {
	new_object_message_received = 1;
	new_left_object = objects->left_blocked;
	new_right_object = objects->right_blocked;
	new_stop = objects->stop_zone_occupied;
	
	object_left_distance	= objects->closest_object_distance_left;
	object_right_distance	= objects->closest_object_distance_right;
	object_left_angle		= objects->closest_object_angle_left;
	object_right_angle		= objects->closest_object_angle_right;
	//ROS_INFO("Object received: Left %d, Right %d, Stop %d", new_left_object, new_right_object, new_stop);
}

void PotDecision::gyroCallback(const fmMsgs::gyroscope::ConstPtr& gyro) {
	//ROS_INFO("Gyro z: %f", gyro->z);
	new_gyro = 1;
	gyro_z = gyro->z;
}

void PotDecision::wiiCallback(const fmMsgs::wii_state::ConstPtr& wii_state) {
	//ROS_INFO("Wii state: %f", wii_state->mode);
	saved_wii_state = wii_state->mode;
}


void PotDecision::timerCallback(const ros::TimerEvent& event) {
	//ROS_INFO("Timerz!");
	if(new_speeds && new_gyro)
	{
		calculate_odometry();
		new_speeds = 0;
		new_gyro = 0;
	}
	
	if(new_object_row)
	{
		new_object_row = 0;
		extract_object_row_data();
	}
	
	run_state_machine();
}

void PotDecision::calculate_twist_from_object_boxes() {
	static double dt = time_s;
	cross_track_error = 0;
	double dist_cte = 0, ang_cte = 0;
	
	// Start driving decision
	if (new_left_object || new_right_object) {
		if(new_left_object && new_right_object)
		{
			dist_cte = (object_left_distance < object_right_distance) ? object_left_distance: object_right_distance;
			ang_cte = (object_left_distance < object_right_distance) ? object_left_angle: object_right_angle;
		} else if(new_left_object) {
			dist_cte 	= object_left_distance;
			ang_cte 	= object_left_angle;
		} else {
			dist_cte 	= object_right_distance;
			ang_cte 	= object_right_angle;
		}
		
		dist_cte = (ang_cte > 0) ? -dist_cte : dist_cte;
		cross_track_error = ((-1 / ang_cte) * (1/(dist_cte * dist_cte)) / cte_weight_distance); 
		
	} else {
		dist_cte = 0;
		ang_cte = 0;
		cross_track_error = 0;
	}
	
	
	if(abs(cross_track_error) > (M_PI / 4))
	{
		cross_track_error = (cross_track_error < 0) ? -M_PI / 4 : M_PI / 4;
	}
	
	double cte_t = cross_track_error;
	//cross_track_error = cte_pid.run(cross_track_error, dt);
	
	// Build twist
	twist_msg.twist.angular.z += cross_track_error;
	
	new_left_object = 0;
	new_right_object = 0;
}


void PotDecision::calculate_odometry() {
	static double x = 0.0, y = 0.0, th = 0.0;
	//static ros::Time last_time = ros::Time::now();
	static double b =  (2 * base_link_radius_to_wheels); // Length between the wheels
	static double gyro_first_value = 1000;
	static double idt = 0;
	static double dt = time_s;
	
	// Initialize gyro to get rid of offset.
	if (gyro_first_value == 1000 && new_gyro) {
		gyro_first_value = gyro_z;
	}
	
	// Start calculations
	//ros::Time current_time = ros::Time::now();
	//double dt = (current_time - last_time).toSec();
	double lv = wheel_speed_left, rv = wheel_speed_right;
	double forward_speed = (lv + rv) / 2.0; // Forward speed in m/s
	
	double wticks = (rv - lv) / b;
	double wgyro = gyro_z - gyro_first_value;
	double w = wticks * 0.2 + wgyro * 0.8;
	double beta = w * dt;
	double distance_travelled = forward_speed * dt; // in m.
	
	//if(abs(beta) > dead_reckoning_linearization_turn_rate_threshold)
	if(abs(beta) > 0.0001)
	{
		// turn_rate above treshold
		double turn_radius = distance_travelled / beta;
		double cx = (x - sin(th) * turn_radius);
		double cy = (y + cos(th) * turn_radius);
		x 	= cx + sin(th + beta) * turn_radius;
		y 	= cy - cos(th + beta) * turn_radius;
	} else {
		// turn_rate below threshold
		x 	= x + distance_travelled * cos(th);
		y 	= y + distance_travelled * sin(th);
	}
	
	
	th	= fmod((th + beta), (2.0 * M_PI));
	
	double th_row = 0;
	//cross_track_error = 0;
	//double dist_cte = 0, ang_cte = 0;
	if(new_l_row && new_r_row)
	{
		th_row = (rightangle + leftangle) / 2;
		th = th * 0.5 + th_row * 0.5;
		//dist_cte = rightdistance - leftdistance;
	} else if(new_l_row){
		th_row = leftangle;
		th = th * 0.5 + th_row * 0.5;
		
		//cross_track_error = (leftdistance - 0.35) + (leftangle);
		//dist_cte = leftdistance - mean_driving_distance_from_rows;
	} else if(new_r_row){
		th_row = rightangle;
		th = th * 0.5 + th_row * 0.5;
		
		//cross_track_error = (rightdistance - 0.35) + (rightangle);
		//dist_cte = rightdistance - mean_driving_distance_from_rows;
	}
	
	// // Start driving decisio
	// if (new_left_object || new_right_object) {
	// 	if(new_left_object && new_right_object)
	// 	{
	// 		dist_cte = (object_left_distance < object_right_distance) ? object_left_distance: object_right_distance;
	// 		ang_cte = (object_left_distance < object_right_distance) ? object_left_angle: object_right_angle;
	// 	} else if(new_left_object) {
	// 		dist_cte 	= object_left_distance;
	// 		ang_cte 	= object_left_angle;
	// 	} else {
	// 		dist_cte 	= object_right_distance;
	// 		ang_cte 	= object_right_angle;
	// 	}
	// } else {
	// 	dist_cte = 0;
	// 	ang_cte = 0;
	// }
	// 
	// 
	// cross_track_error = dist_cte * cte_weight_distance -ang_cte * cte_weight_angle;
	// double cte_t = cross_track_error;
	// cross_track_error = cte_pid.run(cross_track_error, dt);
	// ROS_INFO("PIDet: %f, NonPIDet: %f", cross_track_error, cte_t);
	// 
	// //ROS_INFO("\t%f\t%f\t%f\t%f\t%f\t%f", x, y, th, idt, wticks, wgyro);
	// // Publish twist to ros
	// ++twist_msg.header.seq;
	// twist_msg.header.stamp = ros::Time::now();
	// twist_msg.twist.linear.x = linear_mean_velocity;
	// twist_msg.twist.angular.z = cross_track_error;
	// 
	// 
	// 
	// if (new_stop) {
	// 	twist_msg.twist.linear.x = 0;
	// 	twist_msg.twist.angular.z = 0;
	// }
	// 
	// 
	// twist_pub.publish(twist_msg);

	
	// Update time
	idt += dt;

	// Values used.
	new_l_row = 0;
	new_r_row = 0;
	// new_stop = 0;
	// 	new_left_object = 0;
	// 	new_right_object = 0;
}
void PotDecision::markerCallback(const std_msgs::Int8::ConstPtr& marker){
	marker_found = 1;
}

/*
 * Main loop
 * Initializes ros, its parameters and the subscribers/publishers
 * Instatiates an object to run the laser_scan callback
 * Creates a handle to OpenCV window frames, if this is specified in the parameters.
 */
int main(int argc, char** argv){
	// Initialize ROS
	ros::init(argc, argv, "PotDecision");
	
	// Ros nodehandles
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
	
	// Instatiate object
	PotDecision pd;
	
	// Ros params
	string wheel_topic, row_topic, gyro_topic, twist_topic, object_topic, object_row_topic, row_state_topic, wii_state_topic;
	
	//nh.param<string>("laser_scan_topic", laser_scan_topic, "laser_scan_topic");
	nh.param<string>("row_topic", row_topic, "row_topic");
	nh.param<string>("wheel_topic", wheel_topic, "wheel_topic");
	nh.param<string>("gyro_topic", gyro_topic, "gyro_topic");
	nh.param<string>("twist_topic", twist_topic, "/cmd_vel");
	nh.param<string>("object_topic", object_topic, "object_topic");
	nh.param<string>("object_row_topic", object_row_topic, "object_row_topic");
	nh.param<string>("row_state_topic", row_state_topic, "row_state_topic");
	nh.param<string>("wii_state_topic", wii_state_topic, "/wii_state_topic");
	nh.param<double>("time_s", pd.time_s, 0.1);
	nh.param<double>("linear_mean_velocity", pd.linear_mean_velocity, 0.5);
	nh.param<double>("mean_driving_distance_from_rows", pd.mean_driving_distance_from_rows, 0.35);
	nh.param<double>("cte_weight_angle", pd.cte_weight_angle, 0.5);
	nh.param<double>("cte_weight_distance", pd.cte_weight_distance, 0.5);
	nh.param<double>("base_link_radius_to_wheels", pd.base_link_radius_to_wheels, 0.185);
	nh.param<double>("cte_kp", pd.cte_kp, 1);
	nh.param<double>("cte_ki", pd.cte_ki, 0);
	nh.param<double>("cte_kd", pd.cte_kd, 0);
	nh.param<int>("object_row_box_filled_threshold", pd.object_row_box_filled_threshold, 0);
	nh.param<double>("object_row_threshold", pd.object_row_threshold, 0.5);
	nh.param<double>("dead_reckoning_turn_rate", pd.dead_reckoning_turn_rate, 0.5);

	//nh.param<double>("max_dist_to_rows", pd.max_dist_to_rows, 0.6);
	//nh.param<int>("show_image", pd.show_image_boolean, 1);
	
	// Subscribes and publishers
	//ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 10, &PotDetector::laserScanCallback, &pd);
	ros::Subscriber row_subscriber = n.subscribe<fmMsgs::row>(row_topic.c_str(), 1, &PotDecision::rowCallback, &pd);
	ros::Subscriber wii_subscriber = n.subscribe<fmMsgs::wii_state>(wii_state_topic.c_str(), 1, &PotDecision::wiiCallback, &pd);
	ros::Subscriber wheel_subscriber = n.subscribe<fmMsgs::float_data>(wheel_topic.c_str(), 10, &PotDecision::wheelCallback, &pd);
	ros::Subscriber gyro_subscriber = n.subscribe<fmMsgs::gyroscope>(gyro_topic.c_str(), 10, &PotDecision::gyroCallback, &pd);
	ros::Subscriber object_subscriber = n.subscribe<fmMsgs::detected_objects>(object_topic.c_str(), 10, &PotDecision::objectCallback, &pd);
	ros::Subscriber object_row_subscriber = n.subscribe<fmMsgs::object_row>(object_row_topic.c_str(), 10, &PotDecision::objectRowCallback, &pd);
	ros::Subscriber marker_sub = n.subscribe<std_msgs::Int8>("/marker_pub", 1, &PotDecision::markerCallback, &pd);

	pd.twist_pub = n.advertise<geometry_msgs::TwistStamped>(twist_topic.c_str(), 1);
	pd.row_state_pub = n.advertise<fmMsgs::hilde_states>(row_state_topic.c_str(), 1);
	
	ros::Timer timer = n.createTimer(ros::Duration(pd.time_s), &PotDecision::timerCallback, &pd);

	pd.cte_pid(pd.cte_kp, pd.cte_ki, pd.cte_kd);
	
	
	// Spin
	ros::spin();
	return 0;
}

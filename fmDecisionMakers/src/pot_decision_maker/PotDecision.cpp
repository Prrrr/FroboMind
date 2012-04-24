/*
 * PotDecision.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: Nis Sarup
 */


#include "PotDecision.h"
using namespace std;
/*
 * Constructor
 */
PotDecision::PotDecision() {
	new_speeds = 0;
	new_gyro = 0;
	new_l_row = 0;
	new_r_row = 0;
}
/*
 * Destructor
 */
PotDecision::~PotDecision() {

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

void PotDecision::timerCallback(const ros::TimerEvent& event) {
	//ROS_INFO("Timerz!");
	if(new_speeds && new_gyro)
	{
		calculate_odometry();
		new_speeds = 0;
		new_gyro = 0;
	}
	
	if(new_object_message_received)
	{
		new_object_message_received = 0;
		calculate_twist();
	}
}

void PotDecision::calculate_twist() {
	//ROS_INFO("Timerz!");
	static double dt = time_s;
	cross_track_error = 0;
	double dist_cte = 0, ang_cte = 0;
	
	// Start driving decisio
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
		
		// dist_cte = 1 / dist_cte;
		// ang_cte = 1 / ang_cte;
		
		dist_cte = (ang_cte > 0) ? -dist_cte : dist_cte;
		//cross_track_error = ((-1 / ang_cte) * (1/(dist_cte * dist_cte)) / 100); 
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
	
	//cross_track_error = dist_cte * cte_weight_distance - ang_cte * cte_weight_angle;
	
	double cte_t = cross_track_error;
	cross_track_error = cte_pid.run(cross_track_error, dt);
	ROS_INFO("PIDet: %f, NonPIDet: %f", cross_track_error, cte_t);
	
	//ROS_INFO("\t%f\t%f\t%f\t%f\t%f\t%f", x, y, th, idt, wticks, wgyro);
	// Publish twist to ros
	++twist_msg.header.seq;
	twist_msg.header.stamp = ros::Time::now();
	twist_msg.twist.linear.x = linear_mean_velocity;
	twist_msg.twist.angular.z = cross_track_error;
	
	
	
	if (new_stop) {
		twist_msg.twist.linear.x = 0;
		twist_msg.twist.angular.z = 0;
	}
	
	
	twist_pub.publish(twist_msg);
	
	new_stop = 0;
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
	string wheel_topic, row_topic, gyro_topic, twist_topic, object_topic;
	
	//nh.param<string>("laser_scan_topic", laser_scan_topic, "laser_scan_topic");
	nh.param<string>("row_topic", row_topic, "row_topic");
	nh.param<string>("wheel_topic", wheel_topic, "wheel_topic");
	nh.param<string>("gyro_topic", gyro_topic, "gyro_topic");
	nh.param<string>("twist_topic", twist_topic, "/cmd_vel");
	nh.param<string>("object_topic", object_topic, "object_topic");
	nh.param<double>("time_s", pd.time_s, 0.1);
	nh.param<double>("linear_mean_velocity", pd.linear_mean_velocity, 0.5);
	nh.param<double>("mean_driving_distance_from_rows", pd.mean_driving_distance_from_rows, 0.35);
	nh.param<double>("cte_weight_angle", pd.cte_weight_angle, 0.5);
	nh.param<double>("cte_weight_distance", pd.cte_weight_distance, 0.5);
	nh.param<double>("base_link_radius_to_wheels", pd.base_link_radius_to_wheels, 0.185);
	nh.param<double>("cte_kp", pd.cte_kp, 1);
	nh.param<double>("cte_ki", pd.cte_ki, 0);
	nh.param<double>("cte_kd", pd.cte_kd, 0);


	//nh.param<double>("max_dist_to_rows", pd.max_dist_to_rows, 0.6);
	//nh.param<int>("show_image", pd.show_image_boolean, 1);
	
	// Subscribes and publishers
	//ros::Subscriber laser_subscriber = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic.c_str(), 10, &PotDetector::laserScanCallback, &pd);
	ros::Subscriber row_subscriber = n.subscribe<fmMsgs::row>(row_topic.c_str(), 1, &PotDecision::rowCallback, &pd);
	ros::Subscriber wheel_subscriber = n.subscribe<fmMsgs::float_data>(wheel_topic.c_str(), 10, &PotDecision::wheelCallback, &pd);
	ros::Subscriber gyro_subscriber = n.subscribe<fmMsgs::gyroscope>(gyro_topic.c_str(), 10, &PotDecision::gyroCallback, &pd);
	ros::Subscriber object_subscriber = n.subscribe<fmMsgs::detected_objects>(object_topic.c_str(), 10, &PotDecision::objectCallback, &pd);

	pd.twist_pub = n.advertise<geometry_msgs::TwistStamped>(twist_topic.c_str(), 1);
	
	ros::Timer timer = n.createTimer(ros::Duration(pd.time_s), &PotDecision::timerCallback, &pd);

	pd.cte_pid(pd.cte_kp, pd.cte_ki, pd.cte_kd);
	
	// Spin
	ros::spin();
	return 0;
}

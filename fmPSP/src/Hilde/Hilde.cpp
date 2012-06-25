/*
 * Hilde.cpp
 *
 *  Created on: Mar 27, 2012
 *      Author: Morten Ege Jensen
 */

#include <ros/ros.h>
#include "math.h"
#include "geometry_msgs/TwistStamped.h"
#include "fmMsgs/serial_bin.h"
#include "fmMsgs/float_data.h"
#include "boost/circular_buffer.hpp"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
	 
using namespace std;
class Hilde {
private:
	//  Node handling
	ros::NodeHandle global_n;
	ros::NodeHandle local_n;
	
	// Subscribers and publishers
	ros::Subscriber twist_subscriber;
	ros::Subscriber robocard_subscriber;
	ros::Publisher robocard_publisher;
	ros::Publisher wheelspeed_publisher;
	
	// ros msgs
	fmMsgs::serial_bin robocard_tx_msg;
	fmMsgs::float_data wheel_speeds_msg;
	
	// Parameters
	string twist_subscriber_topic;
	string robocard_subscriber_topic;
	string robocard_publisher_topic;
	string wheel_speed_topic;
	double wheel_radius;
	double max_velocity;
	double icr_radius_to_wheels;
	int encoder_circular_buffer_size;
	double encoder_dt; // How often new ticks are received from the robocard
	double base_link_length_to_rear_wheel;
	double base_link_radius_to_wheels; // Distance from center of robot to the wheel
	double dead_reckoning_linearization_turn_rate_threshold;
	
	// Callback Functions
	void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void serialCallback(const fmMsgs::serial_bin::ConstPtr& msg);
	// Functions
	void calculateMotorFromTwist(double linear_vel, double angular_vel);
	double check_velocity(double vel);
	void publish_odom(double lv, double rv);
	

	// Circular buffers for the encoders
	boost::circular_buffer<int> left_encoder_ticks;
	boost::circular_buffer<int> right_encoder_ticks;
	
	
public:
	Hilde();
	virtual ~Hilde();
};

struct protocol{
	unsigned char length;
	vector<unsigned char> data;
	unsigned char flags;
	unsigned char checksum(){
		unsigned char checksum = length;
		for (int i = 0; i < length; i++){
			checksum += data[i];
		}
		checksum += flags;
		return checksum;
	}

	vector<unsigned char> result(){
		vector<unsigned char> results;
		results.push_back(length);
		for (int i = 0; i < data.size(); i++)
			results.push_back(data[i]);
		results.push_back(flags);
		results.push_back(checksum());
		return results;
	}
};

Hilde::Hilde() {
	global_n = ros::NodeHandle();
	local_n = ros::NodeHandle("~");
	//  Get parameters from parameter server if it's available, otherwise set to default values.
	local_n.param<std::string>("twist_subscriber_topic", twist_subscriber_topic, "/cmd_vel");
	local_n.param<std::string>("serial_subscriber_topic", robocard_subscriber_topic, "S0_rx_topic");
	local_n.param<std::string>("serial_publisher_topic", robocard_publisher_topic, "S0_tx_topic");
	local_n.param<std::string>("wheel_speed_topic", wheel_speed_topic, "wheel_speed_topic");
	local_n.param<double>("wheel_radius", wheel_radius, 0.085);
	local_n.param<double>("icr_radius_to_wheels", icr_radius_to_wheels, 0.185);
	local_n.param<double>("max_velocity", max_velocity, 0.75);
	local_n.param<int>("encoder_circular_buffer_size", encoder_circular_buffer_size, 10);
	local_n.param<double>("encoder_dt", encoder_dt, 0.02);
	local_n.param<double>("base_link_length_to_rear_wheel", base_link_length_to_rear_wheel, 0.28);
	local_n.param<double>("base_link_radius_to_wheels", base_link_radius_to_wheels, 0.02);
	local_n.param<double>("dead_reckoning_linearization_turn_rate_threshold", dead_reckoning_linearization_turn_rate_threshold, 0.01);
	
	// Subscribers and publisher
	twist_subscriber = global_n.subscribe<geometry_msgs::TwistStamped>(twist_subscriber_topic.c_str(), 1, &Hilde::twistCallback,this);
	robocard_subscriber = global_n.subscribe<fmMsgs::serial_bin>(robocard_subscriber_topic.c_str(), 5, &Hilde::serialCallback, this);
	robocard_publisher = global_n.advertise<fmMsgs::serial_bin>(robocard_publisher_topic.c_str(), 1);
	wheelspeed_publisher = global_n.advertise<fmMsgs::float_data>(wheel_speed_topic.c_str(), 1);
	
	// Instantiate circular buffers for the encoders
	left_encoder_ticks = boost::circular_buffer<int>(encoder_circular_buffer_size);
	right_encoder_ticks = boost::circular_buffer<int>(encoder_circular_buffer_size);
	//	Initialize buffers
	for (int i = 0; i < encoder_circular_buffer_size; i++)
	{
		left_encoder_ticks.push_back(0);
		right_encoder_ticks.push_back(0);
	}
	
	
}

void Hilde::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
	double lin, ang;
	lin = msg->twist.linear.x;
	ang = msg->twist.angular.z;
	calculateMotorFromTwist(lin, ang);
}

void Hilde::serialCallback(const fmMsgs::serial_bin::ConstPtr& msg){
	
	// Adding new values to the circular buffers
	left_encoder_ticks.push_back((int)msg->data[1]);
	right_encoder_ticks.push_back((int)msg->data[2]);
	
	// Sum up ticks
	int suml = 0, sumr = 0;
	for (int i = 0; i < encoder_circular_buffer_size; i++) {
		suml += left_encoder_ticks[i];
		sumr += right_encoder_ticks[i];
	}
	
	// Average ticks per encoder_dt
	double aticksl = (double)suml / (encoder_circular_buffer_size);
	double aticksr = (double)sumr / (encoder_circular_buffer_size);
	
	//ROS_INFO("Average ticks left: %f Right: %f", aticksl, aticksr);
	
	// Calculating actual velocity, m/s
	double avell = ((aticksl / 512) * (wheel_radius * 2 * M_PI)) / encoder_dt;
	double avelr = ((aticksr / 512) * (wheel_radius * 2 * M_PI)) / encoder_dt;
	
	// Calculate odometry
	//publish_odom(avell, avelr);
	
	wheel_speeds_msg.header.stamp = ros::Time::now();
	++wheel_speeds_msg.header.seq;
	wheel_speeds_msg.size = 2;
	wheel_speeds_msg.data.clear();
	wheel_speeds_msg.data.push_back(avelr);
	wheel_speeds_msg.data.push_back(avell);
	
	wheelspeed_publisher.publish(wheel_speeds_msg);
	
	//ROS_INFO("Left/right: %f, %f", avell, avelr);
}

// Published odometry-message
// Takes the velocity of the left and right wheel as parameters
void Hilde::publish_odom(double lv, double rv) {
	static double x = 0.0, y = 0.0, th = 0.0;
	static ros::Time last_time = ros::Time::now();
	static double b =  (2 * base_link_radius_to_wheels); // Length between the wheels
	
	// Start calculations
	ros::Time current_time = ros::Time::now();
	//double dt = (current_time - last_time).toSec();

	double dt = 0.02;
	
	double forward_speed = (lv + rv) / 2.0; // Forward speed in m/s
	//double turn_rate = (lv + rv) / b; // Rad/s
	double w = (rv - lv) / b;
	double beta = w * dt;
	double distance_travelled = forward_speed * dt; // in m.
	
	if(abs(beta) > dead_reckoning_linearization_turn_rate_threshold)
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
	
	th	= fmod(abs((th + beta)), (2.0 * M_PI));
		
	last_time = current_time;
	
	ROS_INFO("%f %f %f %f %f", lv, rv, x, y, th);
}

double Hilde::check_velocity(double vel){
	double return_value = fabs(vel);
	if (vel > max_velocity){
		return_value = max_velocity;
	}
	return return_value;
}

void Hilde::calculateMotorFromTwist(double linear_vel, double angular_vel){
	double V, w;
	int output_left, output_right, out_rear;
	double left_velocity, right_velocity;
	double r = wheel_radius;
	double b = icr_radius_to_wheels;
	// transform linear vector (0 - 1) relative to max_speed and set omega (w)
	V = max_velocity * linear_vel;
	w = -angular_vel;
	// Calculate individual wheel velocities
	// See the latex document for clarification
	left_velocity = V/r - b/(2*r)*w;
	right_velocity = V/r + b/(2*r)*w;
	left_velocity = left_velocity * r;
	right_velocity = right_velocity * r;
	// double check the velocities consistent with max_velocity
	left_velocity = check_velocity(left_velocity);
	right_velocity = check_velocity(right_velocity);
	// Calculate velocity relative to 8-bits
	double temp_output_left = left_velocity / max_velocity * 127;
	double temp_output_right = right_velocity / max_velocity * 127;
	output_left = (int) temp_output_left;
	output_right = (int) temp_output_right;
	
	
	
	double temp_rear_wheel_angle = (atan2(w * base_link_length_to_rear_wheel, V) * 180) / M_PI;
	
	//ROS_INFO("Rear wheel calc: w: %f, bs: %f, V: %f, Rear angle: %f", w, base_link_length_to_rear_wheel, V, temp_rear_wheel_angle);
	
	out_rear = (int)(128 - temp_rear_wheel_angle);
	
	//ROS_INFO("V: %f out: %f out_char %d, Rear wheel angle: %f, rear output: %i", V, temp_output_left, output_left, temp_rear_wheel_angle, out_rear);
	
	
	
	// Create protocol
	protocol prot;
	prot.length = 2;
	prot.flags = 0;
	prot.data.push_back(output_left);
	prot.data.push_back(output_right);

	// Create a ROS message and publish
	++robocard_tx_msg.header.seq;
	robocard_tx_msg.header.stamp = ros::Time::now();
	robocard_tx_msg.data.clear();
	vector<unsigned char> temp = prot.result();
	for (int i = 0; i < temp.size(); i++){
		//ROS_INFO("Pushng back: %d", temp[i]);
		robocard_tx_msg.data.push_back(temp[i]);
	}
	robocard_tx_msg.length = temp.size();
	robocard_publisher.publish(robocard_tx_msg);

}
Hilde::~Hilde() {

}

int main(int argc, char** argv){
	ros::init(argc, argv, "Hilde");
	Hilde hilde;
	ros::spin();
	return 0;
}

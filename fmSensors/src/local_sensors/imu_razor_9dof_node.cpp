#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/imu_razor_9dof.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>

ros::Publisher imu_pub;
ros::Publisher imu_commands_pub;
fmMsgs::imu_razor_9dof imu_msg;
fmMsgs::serial imu_command_msg;

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

void send_command_to_imu(std::string c);
void set_imu_enabled();
void set_imu_disabled();
void set_imu_data_verbose();

void imuCallback(const fmMsgs::serial::ConstPtr& msg) {
	
	// imu_msg.header = msg->header;
	// 
	// imu_pub.publish(imu_msg);
	
	int part_count = 0;
	
	// Split the incoming string
	boost::char_separator<char> sep("#=,");
    tokenizer::iterator tok_iter; 
    tokenizer tokens(msg->data, sep);
	
	// How many parts do we have?
    for (tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter) {
    	part_count++;
    }
	
	// A correctly formed data-string should concist of 4 parts
	if(part_count == 4)
	{
		// Start the iterator from the beginning
		tok_iter = tokens.begin(); 
		
		// If the first part is "YPR" then we need to get the IMU setup to send us the raw data
		std::string data_header = *tok_iter++;
			
		if(!strcmp(data_header.c_str(), "YPR"))
		{
			// Set IMU to verbose
			set_imu_data_verbose();
			
			// Enable IMU
			set_imu_enabled();
			
		} else {
			// We should be in the right state now.
			
		}
	} else {
		ROS_DEBUG("Malformed datastring from IMU: %d parts found.", part_count);
	}
}

void set_imu_data_verbose() {
	ROS_INFO("IMU data set to verbose.");
	send_command_to_imu("#os");
}

void set_imu_enabled() {
	ROS_INFO("Enabling IMU.");
	send_command_to_imu("#o1");
}

void set_imu_disabled() {
	ROS_INFO("Disabling IMU.");
	send_command_to_imu("#o0");
}

void send_command_to_imu(std::string c) {
	imu_command_msg.data = c;
	imu_commands_pub.publish(imu_command_msg);
	ROS_INFO("Command send to IMU: %s", imu_command_msg.data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_razor_9dof_parser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;
	std::string imu_command_topic_id;

	n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "/fmBSP/imu_raw_input");
	n.param<std::string> ("publish_topic_id", publish_topic_id, "/fmSensors/imu_razor_9dof_msg");
	n.param<std::string> ("imu_command_topic_id", imu_command_topic_id, "/fmBSP/imu_commands");

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, imuCallback);
	imu_pub = n.advertise<fmMsgs::imu_razor_9dof> (publish_topic_id, 1);
	imu_commands_pub = n.advertise<fmMsgs::serial> (imu_command_topic_id, 1);
	
	ros::spin();
	return 0;
}


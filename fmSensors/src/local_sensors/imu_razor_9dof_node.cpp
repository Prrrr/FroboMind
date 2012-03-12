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

double acc_x;	// Accelerometer, x-axis
double acc_y;	// Accelerometer, y-axis
double acc_z;	// Accelerometer, z-axis
double gyr_x;	// Gyroscope, x-axis
double gyr_y;	// Gyroscope, y-axis
double gyr_z;	// Gyroscope, z-axis
double com_x;	// Compass, x-axis
double com_y;	// Compass, y-axis
double com_z;	// Compass, z-axis

int data_state = 0; // 0: We have not gotten any data yet.
					// 1: Acceleromter filled.
					// 2: Compass filled.
					// 3: Gyroscope filled.

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

void send_command_to_imu(std::string c);
void set_imu_enabled();
void set_imu_disabled();
void set_imu_data_verbose();
void publish_data(const fmMsgs::serial::ConstPtr& msg);
double string_to_double(std::string);

void imuCallback(const fmMsgs::serial::ConstPtr& msg) {
	
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
			
			int data_failure = 0;
			
			switch (data_state) {
				case 0:		// Wait for the first accelerometer data-string
				if(!strcmp(data_header.c_str(), "ACC"))
				{
					std::string tok =  *tok_iter++;
					boost::trim(tok);
					try {
						acc_x = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						 ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter++;
					boost::trim(tok);
					try {
						acc_y = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter;
					boost::trim(tok);
					try {
						acc_z = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					
					if(data_failure)
					{
						// Revert to state 0 and start on a new, fresh, frame
						data_state = 0;
						data_failure = 0;
					} else {
						data_state++;	
					}
				}
				break;
				
				case 1:
				if(!strcmp(data_header.c_str(), "MAG"))
				{
					std::string tok =  *tok_iter++;
					boost::trim(tok);
					try {
						com_x = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter++;
					boost::trim(tok);
					try {
						com_y = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter;
					boost::trim(tok);
					try {
						com_z = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					
					if(data_failure)
					{
						// Revert to state 0 and start on a new, fresh, frame
						data_state = 0;
						data_failure = 0;
					} else {
						data_state++;	
					}
				}
				break;
				
				case 2:
				if(!strcmp(data_header.c_str(), "GYR"))
				{
					std::string tok =  *tok_iter++;
					boost::trim(tok);
					try {
						gyr_x = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter++;
					boost::trim(tok);
					try {
						gyr_y = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					tok =  *tok_iter;
					boost::trim(tok);
					try {
						gyr_z = boost::lexical_cast<double>(tok);
					} catch (boost::bad_lexical_cast const&) {
						ROS_WARN("IMU Data: Bad lexical cast on: \"%s\", data header = \"%s\" state = \"%d\", string = ", tok.c_str(), data_header.c_str(), data_state);
						 data_failure = 1;
					}
					
					if(data_failure)
					{
						// Revert to state 0 and start on a new, fresh, frame
						data_failure = 0;
					} else {
						// Publish data
						publish_data(msg);
					}
					data_state = 0;	
				}
				break;
				
				default:
					ROS_WARN("IMU Data out of sync. Data header: \"%s\", state = \"%d\"", data_header.c_str(), data_state);
				break;
			}
			
		}
	} else {
		ROS_WARN("Malformed datastring from IMU: %d parts found.", part_count);
	}
}

void publish_data(const fmMsgs::serial::ConstPtr& msg) {
	imu_msg.header = msg->header;
	imu_msg.acc_x = acc_x;
	imu_msg.acc_y = acc_y;
	imu_msg.acc_z = acc_z;
	imu_msg.gyr_x = gyr_x;
	imu_msg.gyr_y = gyr_y;
	imu_msg.gyr_z = gyr_z;
	imu_msg.com_x = com_x;
	imu_msg.com_y = com_y;
	imu_msg.com_z = com_z;
	imu_pub.publish(imu_msg);
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
	ROS_DEBUG("Command send to IMU: %s", imu_command_msg.data.c_str());
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

	ros::Subscriber sub = nh.subscribe(subscribe_topic_id.c_str(), 10, imuCallback);
	imu_pub = nh.advertise<fmMsgs::imu_razor_9dof> (publish_topic_id.c_str(), 1);
	imu_commands_pub = nh.advertise<fmMsgs::serial> (imu_command_topic_id.c_str(), 1);
	
	ros::spin();
	return 0;
}


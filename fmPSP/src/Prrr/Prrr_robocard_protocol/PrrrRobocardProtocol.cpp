/*
 * PrrrRobocardProtocol.cpp
 *
 *  Created on: Feb 15, 2012
 *      Author: Morten Ege Jensen
 */

#include "PrrrRobocardProtocol.h"
/*********************************************************************
 * TODO: The checksum is now only for the data. The Checksum is more effective if more are checked.
 */
PrrrRobocardProtocol::PrrrRobocardProtocol() {
	std::string slip_tx_topic;
	std::string slip_rx_topic;
	std::string data_tx_topic;
	std::string data_rx_topic;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("slip_rx", slip_rx_topic, "slip_rx_topic");
	n.param<std::string> ("slip_tx", slip_tx_topic, "slip_tx_topic");
	n.param<std::string> ("data_rx", data_rx_topic, "data_rx_topic");
	n.param<std::string> ("data_tx", data_tx_topic, "data_tx_topic");

	// Publishers and Subscribers
	slip_rx = nh.subscribe<fmMsgs::serial_bin>(slip_rx_topic.c_str(), 1000, &PrrrRobocardProtocol::slipCallback, this);
	data_rx = nh.subscribe<fmMsgs::prrr_protocol>(data_rx_topic.c_str(), 1000, &PrrrRobocardProtocol::dataCallback, this);

	slip_tx = nh.advertise<fmMsgs::serial_bin>(slip_tx_topic.c_str(), 1000);
	data_tx = nh.advertise<fmMsgs::prrr_protocol>(data_tx_topic.c_str(), 1000);
}

/************************************************************************************
 * When a SLIP protocol pushes a msg through.
 * Check Length. If wrong, set flag
 * Convert to prrr protocol format
 * Perform Checksum check. If wrong checksum, set flag
 * Publish
 */
void PrrrRobocardProtocol::slipCallback(const fmMsgs::serial_bin::ConstPtr& msg){
	data_tx_msg.data.clear();
	// Perform checks
	uint8_t calc_checksum = 0, data_checksum = 0;
	uint8_t flags = 0;
	uint8_t msg_length = msg->length;
	uint8_t data_length = msg->data[0];
	ROS_INFO("Protocol RCV: %d %d %d %d %d", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
	// Check lengths
	if (msg_length != (data_length + 3))		// TODO: Something is wrong with the message - Set FLAG_LENGTH_ISSUE
		ROS_INFO("There's a length issue: %d - %d", msg_length, data_length);
	data_tx_msg.len = data_length;
	// Calculate checksum
	for (uint8_t i = 0; i < msg->length-1; i++){
		calc_checksum += msg->data[i];
	}
	// Copy the data - NOTE: ONLY IF THE DATA ARE CONCISE!!!
	for (uint8_t i = 1; i < data_length+1; i++){
		data_tx_msg.data.push_back(msg->data[i]);
		//calc_checksum += msg->data[i];		// TODO: The checksum should calculate len and flags too
	}
	// Check the checksum
	data_checksum = msg->data[data_length+2];
	if (calc_checksum != data_checksum)		// TODO: Something is wrong with the message - Set FLAG_CHECKSUM_ERROR
		ROS_INFO("Checksum error: %d != %d", calc_checksum, data_checksum);
	// Check the FLAGS
	flags |= msg->data[data_length+1];
	data_tx_msg.flags = flags;
	// Publish message to ros
	++data_tx_msg.header.seq;
	ros::Time start = ros::Time::now();
	data_tx_msg.header.stamp = start;
	data_tx.publish(data_tx_msg);
}
/***********************************************************************************
 * When a motor controller node pushes a msg through.
 * Calculate Checksum
 * Convert to slip protocol format
 * Publish
 */
void PrrrRobocardProtocol::dataCallback(const fmMsgs::prrr_protocol::ConstPtr& msg){
	slip_tx_msg.data.clear();
	uint8_t checksum = 0;
	slip_tx_msg.data.push_back(msg->len);
	for (int i = 0; i < msg->len; i++){
		slip_tx_msg.data.push_back(msg->data[i]);
		//checksum += msg->data[i];		// TODO: This Checksum should include len and flags too
	}
	slip_tx_msg.data.push_back(msg->flags);
	// Calculate checksum
	for (int i = 0; i < slip_tx_msg.data.size(); i++){
		checksum += slip_tx_msg.data[i];
	}
	slip_tx_msg.data.push_back(checksum);
	slip_tx_msg.length = msg->data.size();
	++slip_tx_msg.header.seq;
	slip_tx_msg.header.stamp = ros::Time::now();
	slip_tx.publish(slip_tx_msg);
}

PrrrRobocardProtocol::~PrrrRobocardProtocol() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Prrr_protocol");
	PrrrRobocardProtocol protocol;
	ros::spin();
	return 0;
}

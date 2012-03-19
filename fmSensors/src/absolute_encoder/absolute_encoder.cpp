/*
 * absolute_encoder.cpp
 * Created on Jan 06, 2012
 * By : Morten Ege Jensen
*/

#include "absolute_encoder.h"

AbsoluteEncoder::AbsoluteEncoder(ros::Publisher& tx_pub){
	ROS_INFO("Absolute Encoder constructor");
	enc_pub = tx_pub;
}

void AbsoluteEncoder::timerCallBack(const ros::TimerEvent& te){
	  // Publish data to ros
	  ++tx_msg.header.seq;
	  tx_msg.data.clear();
 	  tx_msg.data.push_back(0);
	  tx_msg.length = tx_msg.data.size();
	  ros::Time start = ros::Time::now();
	  tx_msg.header.stamp = start;
	  enc_pub.publish(tx_msg);
}

void AbsoluteEncoder::callBackHandle(const fmMsgs::serial_bin::ConstPtr& msg){
	// Use callback function to distribute data to system
	int pos = (msg->data[0]<<8)+msg->data[1];
//	ROS_INFO("Encoder: %x %x", msg->data[0], msg->data[1]);
	ROS_INFO("Encoder: %d", pos);
}

AbsoluteEncoder::~AbsoluteEncoder(){
	// Terminates class
}


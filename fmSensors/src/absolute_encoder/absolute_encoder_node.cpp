/*
 * absolute_encoder.cpp
 * Created on Jan 06, 2012
 * By : Morten Ege Jensen
*/

#include "ros/ros.h"
#include "absolute_encoder.h"

int main(int argc, char **argv){
	// Initialize parameters
	std::string encoder_pub_topic;
	std::string encoder_sub_topic;
	double time;
	// ROS initialization
	ros::init(argc, argv, "encoder_node");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	// ROS parameters
	n.param<std::string>("encoder_pub_topic", encoder_pub_topic, "encoder_pub_topic");
	n.param<std::string>("encoder_sub_topic", encoder_sub_topic, "encoder_sub_topic");
	n.param<double>("time", time, 0.1);

	// publisher init
	ros::Publisher tx_pub = nh.advertise<fmMsgs::serial_bin>(encoder_pub_topic.c_str(), 1000);

	// instantiate encoder class
	AbsoluteEncoder encoder(tx_pub);
	// Set up timer event and callback
	ros::Duration duration(time);
	ros::Timer timer = nh.createTimer(duration, &AbsoluteEncoder::timerCallBack, &encoder);
	// Add subscriber to incoming msg
	ros::Subscriber enc_sub = nh.subscribe<fmMsgs::serial_bin>(encoder_sub_topic.c_str(), 1000, &AbsoluteEncoder::callBackHandle, &encoder);
	// SPIN - (Could be asynchronous multiple-thread spin)
	ros::spin();
	return 0;
}

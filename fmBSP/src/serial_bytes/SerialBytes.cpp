/*
 * SerialBytes.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: Morten Ege Jensen
 */

#include "SerialBytes.h"
using namespace std;
/***************************************************************
 * Constructor
 */
SerialBytes::SerialBytes() : serial_(io_){
	string pub_topic;
	string sub_topic;
	string dev;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<string> ("pub_topic", pub_topic, "pub_topic");
	n.param<string> ("sub_topic", sub_topic, "sub_topic");
	n.param<string> ("device", dev, "/dev/ttyUSB0");
	n.param<int>("baudrate", baudrate, 38400);
	n.param<int>("bytes", bytes, 7);
	// Resize the buffer to make it work with boost::buffer
	buffer.resize(bytes);
	// Open device
	openDevice(dev, baudrate);
	// Publishers / Subscribers
	serial_sub = nh.subscribe<fmMsgs::serial_bin>(sub_topic.c_str(), 1000, &SerialBytes::writeHandler, this);
	serial_pub = nh.advertise<fmMsgs::serial_bin>(pub_topic.c_str(), 1000);
}


/***********************************************************************
 * Read from serial stream to buffer, callback function when read
 */
void SerialBytes::readSome(){
	if (!ros::ok())
		return;
	boost::asio::async_read(
			serial_,
			boost::asio::buffer(buffer),
			boost::asio::transfer_at_least(bytes),
	        boost::bind(
	        &SerialBytes::readHandler, this,
	        boost::asio::placeholders::error,
	        boost::asio::placeholders::bytes_transferred));
}
/***********************************************************************
 * Callback function for the serial interface
 */
void SerialBytes::readHandler(const boost::system::error_code& error, size_t bytes_transferred){
	if (!bytes_transferred)
		return;
	serial_msg.data.clear();
	++serial_msg.header.seq;
	serial_msg.header.stamp = ros::Time::now();

	for (int i = 0; i < buffer.size(); i++){
		serial_msg.data.push_back(buffer.at(i));
	}
	// for vector clear
	//buffer.clear();
	serial_msg.length = serial_msg.data.size();
	serial_pub.publish(serial_msg);
	readSome();
}
/***********************************************************************
 * callback function for ros topic
 */
void SerialBytes::writeHandler(const fmMsgs::serial_bin::ConstPtr& msg){
	if (serial_.is_open()) {
	serial_.write_some(boost::asio::buffer(msg->data,msg->length));
	}
}
/*************************************************************************
 *	Opens specified device with specified baudrate
 *	return false for opened device
 */
bool SerialBytes::openDevice(string device, int baud){
	try {
		boost::asio::serial_port_base::baud_rate BAUD(baud);
		serial_.open(device);
		serial_.set_option(BAUD);
	} catch (boost::system::system_error &e) {
		ROS_ERROR("Connection to device %s failed; %s",e.what(), device.c_str());
		return 1;
	}
	/* start the read from the serial device */
	readSome();

	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
	return 0;
}

SerialBytes::~SerialBytes() {
    serial_.cancel();
    serial_.close();
}

int main(int argc, char** argv){
	ros::init(argc,argv,"serial_bytes_node");

	SerialBytes serial;

	ros::spin();
	return 0;
}

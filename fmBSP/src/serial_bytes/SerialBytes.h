/*
 * SerialBytes.h
 *
 *  Created on: Feb 20, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef SERIALBYTES_H_
#define SERIALBYTES_H_

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/asio/read.hpp>

#include <ros/ros.h>
#include <fmMsgs/serial_bin.h>
#include <vector>

using namespace std;

class SerialBytes {
private:
	// BOOST
	boost::asio::io_service io_;
	boost::asio::serial_port serial_;
	// ROS
	fmMsgs::serial_bin serial_msg;
	ros::Publisher serial_pub;
	ros::Subscriber serial_sub;
	// TODO: This should not be a constant size
	vector<uint8_t> buffer;
	//boost::array<uint8_t, 7> buffer;
	int baudrate;
	int bytes;
	void writeHandler(const fmMsgs::serial_bin::ConstPtr& msg);
	void readHandler(const boost::system::error_code& error, size_t bytes_transferred);
	bool openDevice(string dev, int baud);
	void readSome();
public:
	SerialBytes();
	virtual ~SerialBytes();
};

#endif /* SERIALBYTES_H_ */

/*
 * SlipProtocol.h
 *
 *  Created on: Feb 8, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef SLIPPROTOCOL_H_
#define SLIPPROTOCOL_H_

#include <ros/ros.h>
#include <fmMsgs/serial_bin.h>
#include <list>

/* SLIP special character codes - in Decimal    */
//#define END             192    /* indicates end of packet 			0300*/
//#define ESC             219    /* indicates byte stuffing 			0333*/
//#define ESC_END         220    /* ESC ESC_END means END data byte 	0334*/
//#define ESC_ESC         221    /* ESC ESC_ESC means ESC data byte 	0335*/
using namespace std;
class SlipProtocol {
private:
	const static char END = 192;
	const static char ESC = 219;
	const static char ESC_END = 220;
	const static char ESC_ESC = 221;

	// ROS
	fmMsgs::serial_bin rx_msg;
	fmMsgs::serial_bin tx_msg;
	fmMsgs::serial_bin wrap_msg;
	fmMsgs::serial_bin unwrapped_msg;
	ros::Publisher slip_tx;
	ros::Publisher slip_unwrapped;
	ros::Subscriber slip_rx;
	ros::Subscriber slip_wrap;
	ros::Timer timer;
	// Buffers
	list<char> slip_tx_buffer;
	list<char> slip_rx_buffer;
	// Functions
	void send_packet(list<char>& buffer);
	int recv_packet(list<char>& buffer, char c);
	void timerCallBack(const ros::TimerEvent& te);
	void callbackReceive(const fmMsgs::serial_bin::ConstPtr& msg);
	void callbackSend(const fmMsgs::serial_bin::ConstPtr& msg);
public:
	SlipProtocol();
	virtual ~SlipProtocol();
};

#endif /* SLIPPROTOCOL_H_ */

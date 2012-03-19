/*
 * PrrrRobocardProtocol.h
 *
 *  Created on: Feb 15, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef PRRRROBOCARDPROTOCOL_H_
#define PRRRROBOCARDPROTOCOL_H_
#include <ros/ros.h>
#include <fmMsgs/prrr_protocol.h>
#include <fmMsgs/serial_bin.h>

using namespace std;
class PrrrRobocardProtocol {
private:
	// ROS Pubs/Subs
	ros::Publisher data_tx;
	ros::Subscriber data_rx;
	ros::Publisher slip_tx;
	ros::Subscriber slip_rx;
	// ROS Msgs
	fmMsgs::prrr_protocol data_tx_msg;
	fmMsgs::serial_bin slip_tx_msg;
	// Functions
	void slipCallback(const fmMsgs::serial_bin::ConstPtr& msg);
	void dataCallback(const fmMsgs::prrr_protocol::ConstPtr& msg);
public:
	PrrrRobocardProtocol();
	virtual ~PrrrRobocardProtocol();
};

#endif /* PRRRROBOCARDPROTOCOL_H_ */

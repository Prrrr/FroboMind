/*
 * PrrrCtrl.h
 *
 *  Created on: Feb 15, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef PRRRCTRL_H_
#define PRRRCTRL_H_
#include <ros/ros.h>
#include <fmMsgs/prrr_protocol.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

class PrrrCtrl {
private:
	fmMsgs::prrr_protocol prrr_msg;
	ros::Subscriber twist_sub;
	ros::Subscriber prrr_sub;
	ros::Publisher prrr_pub;
	void calculateCtrl(double lin, double ang);
	void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void prrrCallback(const fmMsgs::prrr_protocol::ConstPtr& msg);
public:
	PrrrCtrl();
	virtual ~PrrrCtrl();
};

#endif /* PRRRCTRL_H_ */

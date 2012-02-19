/*
 * PrrrCtrl.cpp
 *
 *  Created on: Feb 15, 2012
 *      Author: Morten Ege Jensen
 */

#include "PrrrCtrl.h"
/*******************************************************************
 * Constructor needs attention
 */
PrrrCtrl::PrrrCtrl() {
	string twist_topic;
	string pub_topic;
	string sub_topic;
	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<string> ("twist_topic", twist_topic, "prrr_twist_topic");
	n.param<string> ("pub_topic", pub_topic, "prrr_pub_topic");
	n.param<string> ("sub_topic", sub_topic, "prrr_sub_topic");

	/* Publishers and subscribers */
	twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(twist_topic.c_str(), 1, &PrrrCtrl::twistCallback, this);
	prrr_sub = nh.subscribe<fmMsgs::prrr_protocol>(sub_topic.c_str(), 1, &PrrrCtrl::prrrCallback, this);
	prrr_pub = nh.advertise<fmMsgs::prrr_protocol>(pub_topic.c_str(), 1);
}

/************************************************************************''
 * Takes the twist values and calculates two speeds for differential steering
 * TODO: This needs adjusting
 */
void PrrrCtrl::calculateCtrl(double lin, double ang){
	static double v_left = 0, v_right = 0, speed = 0;
	const double turn_constant = 0.03;
	//Calculate speed for each wheel
	v_left = lin + turn_constant*(ang)/10;
	v_right = lin - turn_constant*(ang)/10;
	// calculate the speed of the vehicle
	speed = (v_left + v_right)/2;
	int8_t left = v_left;
	int8_t right = v_right;
	ROS_INFO("RCV: left %d right %d", left, right);
	// Publish message
	prrr_msg.data.clear();
	++prrr_msg.header.seq;
	prrr_msg.header.stamp = ros::Time::now();
	// We know were only sending 2 bytes of data
	prrr_msg.len = 2;
	prrr_msg.data.push_back((int8_t)v_left);
	prrr_msg.data.push_back((int8_t)v_right);
	// TODO: What flags need to be set?
	prrr_msg.flags = 0;
	prrr_pub.publish(prrr_msg);
}

/********************************************************************************
 * When a Twist msg is received
 * Run the calculate function to post onwards
 */
void PrrrCtrl::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
	double lin = (msg->twist.linear.x) * 100;
	double ang = (msg->twist.angular.x) * 100;
	calculateCtrl(lin, ang);
}

/*************************************************************************
 * TODO: What to do when a msg is received over the UART!!
 * Some Flag-handling has to be here as well
 */
void PrrrCtrl::prrrCallback(const fmMsgs::prrr_protocol::ConstPtr& msg){
	ROS_INFO("len: %d, data %d %d, flags %d",msg->len, msg->data[0], msg->data[1], msg->flags);
}

/************************************************
 * Destructor
 */
PrrrCtrl::~PrrrCtrl() {
	// TODO Auto-generated destructor stub
}

/*****************************************************************
 * Main
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "Prrr_Robocard_ctrl_node");
	PrrrCtrl ctrl;
	ros::spin();
	return 0;
}

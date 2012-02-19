/*
 * SlipProtocol.cpp
 *
 *  Created on: Feb 8, 2012
 *      Author: Morten Ege Jensen
 */

#include "SlipProtocol.h"
/* SLIP state machine defines */
#define SLIP_STATE_IDLE 	0
#define SLIP_STATE_ESC 		1
#define SLIP_STATE_DATA		2

/**********************************************************************
 * The constructor needs attention
 */
SlipProtocol::SlipProtocol() {
	std::string slip_rx_topic;
	std::string slip_tx_topic;
	std::string slip_wrap_topic;
	std::string slip_unwrapped_topic;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("transmit_topic", slip_rx_topic, "slip_tx_msg");
	n.param<std::string> ("receive_topic", slip_tx_topic, "slip_tx_msg");	// TODO: These are looping. NOTICE!!
	n.param<std::string> ("wrap_topic", slip_wrap_topic, "slip_unwrapped_msg"); // TODO: Theese are looping . slip_wrap_msg
	n.param<std::string> ("unwrapped_topic", slip_unwrapped_topic, "slip_unwrapped_msg");


	// This is the timer. Responsible for testing
	// USE this for testing
	ros::Duration duration(1);
	timer = nh.createTimer(duration, &SlipProtocol::timerCallBack, this);

	// Outgoing communication
	// ie. to a MicroController
	slip_rx = nh.subscribe<fmMsgs::serial_bin>(slip_rx_topic.c_str(), 1000, &SlipProtocol::callbackReceive, this);
	slip_tx = nh.advertise<fmMsgs::serial_bin>(slip_tx_topic.c_str(), 1000);
	// Internal communication
	// ie. to another node
	slip_unwrapped = nh.advertise<fmMsgs::serial_bin>(slip_unwrapped_topic.c_str(), 1000);
	slip_wrap = nh.subscribe<fmMsgs::serial_bin>(slip_wrap_topic, 1000, &SlipProtocol::callbackSend, this);
}

/**********************************************************************
 * Callback function for internal msg to be published/ sent to the serial uart connection
 */
void SlipProtocol::callbackSend(const fmMsgs::serial_bin::ConstPtr& msg){
	slip_tx_buffer.clear();
	for (int i = 0; i < msg->data.size(); i++){
		slip_tx_buffer.push_back(msg->data[i]);
	};
	send_packet(slip_tx_buffer);
}
/****************************************************************
 * Receives msgs formatted to the SLIP protocol (ie. over UART)
 * and publishes them as raw data
 * TODO: This receiver ASSUMES that only 1 byte is received from the serial port at a time
 */
void SlipProtocol::callbackReceive(const fmMsgs::serial_bin::ConstPtr& msg){
	if ((int)recv_packet(slip_rx_buffer, msg->data[0])){	// This assumes only one character over the line at a time
		++unwrapped_msg.header.seq;
		unwrapped_msg.data.clear();
		while (!slip_rx_buffer.empty()){					// Cleares the buffer again as well
			unwrapped_msg.data.push_back(slip_rx_buffer.front());
			slip_rx_buffer.pop_front();
		}

		unwrapped_msg.length = unwrapped_msg.data.size();
		ros::Time start = ros::Time::now();
		unwrapped_msg.header.stamp = start;
		slip_unwrapped.publish(unwrapped_msg);
	}
}
/**********************************************************
 * This method is at the moment only for testing the loop-back
 * TODO: Revise its need when committing
 */
void SlipProtocol::timerCallBack(const ros::TimerEvent& te){
	unwrapped_msg.data.clear();
	unwrapped_msg.header.stamp = ros::Time::now();
	++unwrapped_msg.header.seq;
	unwrapped_msg.data.push_back(2);
	unwrapped_msg.data.push_back(4);
	unwrapped_msg.data.push_back(4);
	unwrapped_msg.data.push_back(7);
	unwrapped_msg.data.push_back(8);
	unwrapped_msg.length = unwrapped_msg.data.size();
	slip_unwrapped.publish(unwrapped_msg);
	/*
	static int cnt = 0;
	if (cnt >= 0){
		cnt++;
		ROS_INFO("TIMER'S SENDING");
		char str[] = {2, 2, 3, 0, 5, END};
		for (int i = 0; i < 6; i++){
			++tx_msg.header.seq;
			tx_msg.data.clear();
	//		tx_msg.data.push_back(END);
	//		tx_msg.data.push_back('a');
	//		tx_msg.data.push_back('b');
	//		tx_msg.data.push_back(END);
			tx_msg.data.push_back(str[i]);
			//ROS_INFO("PUSHED: %x", str[i]);
			tx_msg.length = tx_msg.data.size();
			ros::Time start = ros::Time::now();
			tx_msg.header.stamp = start;
			slip_tx.publish(tx_msg);
		}
	}

	//list<char> l;
	//l.push_back('a');
	//l.push_back('b');
	//send_packet(l);
	//l.push_back(END);
	//l.push_back('c');
 */
}

/********************************************************************
 * Generic SLIP function to receive one char and add to the buffer specified by buffer
 * Return TRUE if a full SLIP packet has been received
 * TODO: Revise if the external buffer-call is necessary
 */
int SlipProtocol::recv_packet(list<char>& buffer, char c){
	static int state = SLIP_STATE_IDLE;
	switch (state){
	case SLIP_STATE_IDLE:
		switch(c){
		case END:
			if (!buffer.empty())
				return 1;
			else
				return 0;
		case ESC:
			state = SLIP_STATE_ESC;
			break;
		default:
			state = SLIP_STATE_DATA;
			break;
		}
	case SLIP_STATE_ESC:
		switch (c){
		case ESC_ESC:
			c = ESC;
			state = SLIP_STATE_DATA;
			break;
		case ESC_END:
			c = END;
			state = SLIP_STATE_DATA;
			break;
		}
	case SLIP_STATE_DATA:
		buffer.push_back(c);
		ROS_INFO("ADDED CHAR %x", c);
		state = SLIP_STATE_IDLE;
		break;
	default:
		state = SLIP_STATE_IDLE;
		break;
	}
	return 0;
}

/********************************************************************
 * Generic slip send packet function
 * TODO: Revise if the external buffer-call is necessary
 */
void SlipProtocol::send_packet(list<char>& buffer){
	char c;
	++tx_msg.header.seq;
	tx_msg.data.clear();
	// Start of SLIP Protocol
	tx_msg.data.push_back(END);
	while(!buffer.empty()){
	  c = buffer.front();
	  buffer.pop_front();
	  //ROS_INFO("WHAT: %x", c);
	  switch(c){
	  case END:
		  //ROS_INFO("PUSHED BACK END CHARACTER!!");
		  tx_msg.data.push_back(ESC);
		  tx_msg.data.push_back(ESC_END);
		  break;
	  case ESC:
		  tx_msg.data.push_back(ESC);
		  tx_msg.data.push_back(ESC_ESC);
		  break;
	  default:
		  tx_msg.data.push_back(c);
		  break;
	  }
	}
	tx_msg.data.push_back(END);
	// End of SLIP Protocol
	tx_msg.length = tx_msg.data.size();
	ros::Time start = ros::Time::now();
	tx_msg.header.stamp = start;

	for (int i = 0; i < (int)tx_msg.data.size(); i++){
	  //ROS_INFO("MSG: %d", tx_msg.data[i]);
	}
	slip_tx.publish(tx_msg);
	//ROS_INFO("WE PUBLISHED");
}

/**
 * Destructor: Not in use
 */
SlipProtocol::~SlipProtocol() {
	// TODO Auto-generated destructor stub
}

/***********************************************************
 * MAIN
 */
int main(int argc, char** argv){
	/* initialize ros usage */
	ros::init(argc, argv, "slip_protocol");

	SlipProtocol slip;

	ros::spin();
}

/*
 * absolute_encoder.h
 * Created on Jan 06, 2012
 * By : Morten Ege Jensen
*/

#ifndef ABSOLUTEENCODER_H_
#define ABSOLUTEENCODER_H_

#include "ros/ros.h"
#include "fmMsgs/serial_bin.h"

class AbsoluteEncoder{
private:
 fmMsgs::serial_bin tx_msg;
 fmMsgs::serial_bin rx_msg;
 ros::Publisher enc_pub;
 ros::Publisher data_pub;

public:

 AbsoluteEncoder(ros::Publisher& tx_pub);
 void timerCallBack(const ros::TimerEvent& te);
 void callBackHandle(const fmMsgs::serial_bin::ConstPtr& msg);
 virtual ~AbsoluteEncoder();

};
#endif /* ABSOLUTEENCODER_H_ */

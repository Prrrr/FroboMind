#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "fmMsgs/serial.h"
#include "fmMsgs/imu_razor_9dof.h"
#include "boost/tokenizer.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/algorithm/string.hpp>

ros::Publisher imu_pub;
fmMsgs::imu_razor_9dof imu_msg;

void imuCallback(const fmMsgs::serial::ConstPtr& msg) {
	
	imu_msg.header = msg->header;
	
	imu_pub.publish(imu_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_razor_9dof_parser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	n.param<std::string> ("subscribe_topic_id", subscribe_topic_id, "/fmBSP/imu_raw_input");
	n.param<std::string> ("publish_topic_id", publish_topic_id, "/fmSensors/imu_razor_9dof_msg");

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, imuCallback);
	imu_pub = n.advertise<fmMsgs::imu_razor_9dof> (publish_topic_id, 1);

	ros::spin();
	return 0;
}


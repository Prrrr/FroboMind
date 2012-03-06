#include <csignal>
#include <cstdio>

#include <boost/asio.hpp>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "AES-25_electric_steering.h"


/* published data */
fmMsgs::can can_tx_msg;

ros::Publisher can_tx_pub;
ros::Subscriber can_rx_sub;
ros::Subscriber steering_sub;
ros::Timer can_tx_timer;

AES25 aes25;


void aes25NodePeriodicCallback(const ros::TimerEvent& e)
{
  can_tx_msg = aes25.processCanTxEvent();
  can_tx_pub.publish(can_tx_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "aes25_node");
  ros::NodeHandle nh("~");

  std::string publisher_topic;
  std::string subscriber_topic;
  std::string steering_subscriber_topic;

  nh.param<std::string>("publisher_topic", publisher_topic, "/fmBSP/can0_tx");
  nh.param<std::string>("subscriber_topic", subscriber_topic, "/fmBSP/can0_rx");
  nh.param<std::string>("steering_angle_cmd", steering_subscriber_topic, "/fmActuators/steering_angle_cmd");

  can_tx_pub = nh.advertise<fmMsgs::can> (publisher_topic.c_str(), 1);
  can_rx_sub = nh.subscribe<fmMsgs::can> (subscriber_topic.c_str(), 1000, &AES25::processCanRxEvent, &aes25);
  steering_sub = nh.subscribe<fmMsgs::steering_angle_cmd> (steering_subscriber_topic.c_str(),10,&AES25::setSteeringWheel,&aes25);

  can_tx_timer = nh.createTimer(ros::Duration(0.02), aes25NodePeriodicCallback);

  ros::spin();

  return 0;
}


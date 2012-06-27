#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "wii_state.h"
#include "wiimote/LEDControl.h"
#include "wiimote/RumbleControl.h"
#include "wiimote/State.h"
#include "fmMsgs/wii_state.h"

int main(int argc, char **argv)
{
	/* initialize ros usage */
	ros::init(argc, argv, "wii_state");

	/* parameters */
	std::string wiimote_sub_topic;
	std::string led_pub_topic;
	std::string rumble_pub_topic;
	std::string state_pub_topic;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("wiimote_sub_topic", wiimote_sub_topic, "/wiimote/state"); //Specify the publisher name
	n.param<std::string> ("led_pub_topic", led_pub_topic, "/wiimote/leds"); //Specify the publisher name
	n.param<std::string> ("rumble_pub_topic", rumble_pub_topic, "/wiimote/rumble"); //Specify the publisher name
	n.param<std::string> ("state_pub_topic", state_pub_topic, "/state");

	WiiState ws = WiiState();
	
	n.param<int> ("start_mode", ws.mode, 0);
	
	
	ws.wiimote_state = n.subscribe(wiimote_sub_topic.c_str(),1,&WiiState::wiimoteHandler,&ws);
	ws.wiimote_led = n.advertise<wiimote::LEDControl>(led_pub_topic.c_str(), 1);
	ws.wiimote_rumble = n.advertise<wiimote::RumbleControl>(rumble_pub_topic.c_str(), 1);
	ws.state_pub = n.advertise<fmMsgs::wii_state>(state_pub_topic.c_str(),1);

	ros::Timer timer = n.createTimer(ros::Duration(0.25), &WiiState::timerCallback, &ws);

	ws.stateLoop();

	return 0;
}

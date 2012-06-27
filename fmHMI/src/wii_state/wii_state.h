#include "geometry_msgs/TwistStamped.h"
//#include "fmMsgs/motor_power.h"
//#include "fmMsgs/odometry.h"
#include "fmMsgs/warhorse_state.h"
#include "ros/ros.h"
#include "wiimote/LEDControl.h"
#include "wiimote/RumbleControl.h"
#include "wiimote/State.h"
#include "wiimote/TimedSwitch.h"

#ifndef WII_STATE_H_
#define WII_STATE_H_

class WiiState
{
private:

	std::vector<bool> buttons;
	std::vector<bool> buttons_old;
	std::vector<bool> buttons_pushed;

	void checkButtons();
	void rumble(double duration = 0.1);
	void led(int l0, int l1, int l2, int l3);

	int state;
	int mode;

	fmMsgs::warhorse_state warhorse_state;

public:

  ros::Subscriber wiimote_state;

  ros::Publisher wiimote_led;
  ros::Publisher wiimote_rumble;
  ros::Publisher state_pub;

  WiiState();

  void stateLoop();
  void wiimoteHandler(const wiimote::StateConstPtr& state);

};

#endif

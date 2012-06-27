#include "wii_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define wiiA		4
#define wiiB		5
#define wiiPlus		2
#define wiiMinus	3
#define wiiHome		10
#define wiiLeft		8
#define wiiRight	9
#define wiiUp		6
#define wiiDown		7
#define wiiOne		0
#define wiiTwo		1

#define ON 1
#define OFF 0
#define NO_CHANGE -2
#define FLASH -1

enum states {
	manual_drive = 0,
	task1left = 1,
	task1right = 2,
	task2 = 3,
	task3 = 4
};

enum modes {
	drive = 10,
	stop = 11,
	menu = 12,
	paused = 13
};

WiiState::WiiState()
{
	buttons.resize(11,false);
	buttons_old.resize(11,false);
	buttons_pushed.resize(11,false);

	mode = start_mode;
	state = manual_drive;
}

void WiiState::wiimoteHandler(const wiimote::StateConstPtr& state)
{
	for (int i = 0; i < state->buttons.size(); i++)
		buttons[i] = state->buttons[i];
}

void WiiState::rumble(double duration)
{
	wiimote::RumbleControl rum;
	rum.rumble.switch_mode = rum.rumble.REPEAT;
	rum.rumble.num_cycles = 1;
	rum.rumble.pulse_pattern.push_back(duration);
	wiimote_rumble.publish(rum);
}

void WiiState::checkButtons()
{
	for (int i = 0; i < buttons.size(); i++)
	{
		if (buttons[i] && !buttons_old[i])
		{
			buttons_pushed[i] = true;
			ROS_INFO("Button %d pushed", i);
		}
		else
			buttons_pushed[i] = false;

		buttons_old[i] = buttons[i];
	}

}

void WiiState::led_single(int nr, int status) {
	static int leds[4] = { 0, 0, 0, 0 };
	leds[nr] = status;
	led(leds[0], leds[1], leds[2], leds[3]);
}

void WiiState::led(int l0, int l1, int l2, int l3)
{
	int leds[4] = { l0, l1, l2, l3 };
	wiimote::LEDControl led;

	for (int i = 0; i < 4; i++)
	{
		wiimote::TimedSwitch temp;
		temp.switch_mode = leds[i];
		temp.pulse_pattern.push_back(0.03);
		temp.pulse_pattern.push_back(0.03);
		temp.num_cycles = temp.FOREVER;
		led.timed_switch_array.push_back(temp);
	}

	wiimote_led.publish(led);
}

void WiiState::timerCallback(const ros::TimerEvent& event) {
	static int status = 0;
	if(status)
	{
		status = 0;
		led_single(3, OFF);
	} else {
		status = 1;
		led_single(3, ON);
	}
	
}

void WiiState::stateLoop()
{
	ros::Rate loop_rate(10); //Encoder loop frequency

	while (ros::ok())
	{
		ros::spinOnce();
		checkButtons();
		
		switch (mode) {
			case stop:
				wii_state_msg.mode = wii_state_msg.STOP;
				if (buttons_pushed[wiiA])
					{
						ROS_INFO("Changing to drive");
						mode = drive;
						led_single(0, ON);
					}
			break;
			
			case drive:
				wii_state_msg.mode = wii_state_msg.DRIVE;;
				if (buttons_pushed[wiiA])
					{
						ROS_INFO("Changing to pause");
						mode = stop;
						led_single(0, OFF);
					}
			break;
			
			default:
			break;
		}

		state_pub.publish(wii_state_msg);

		loop_rate.sleep();
	}
}


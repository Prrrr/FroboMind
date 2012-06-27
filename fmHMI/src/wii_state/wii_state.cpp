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

	mode = menu;
	state = manual_drive;

	warhorse_state.drive_state = warhorse_state.STOP;
	warhorse_state.task_state = warhorse_state.MANUAL_DRIVE;
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

void WiiState::stateLoop()
{
	ros::Rate loop_rate(10); //Encoder loop frequency

	while (ros::ok())
	{
		ros::spinOnce();
		checkButtons();

		switch (mode) {
			case menu:
				switch (state) {
					case manual_drive:
						led(OFF,ON,ON,ON);
						warhorse_state.task_state = warhorse_state.MANUAL_DRIVE;
						if (buttons_pushed[wiiPlus])
							state = task1left;
						if (buttons_pushed[wiiMinus])
							state = task3;
						if (buttons_pushed[wiiHome])
						{
							mode = drive;
							led(ON,OFF,OFF,OFF);
						}
						break;
					case task1left:
						led(ON,OFF,ON,ON);
						warhorse_state.task_state = warhorse_state.TASK1LEFT;
						if (buttons_pushed[wiiPlus])
							state = task1right;
						if (buttons_pushed[wiiMinus])
							state = manual_drive;
						if (buttons_pushed[wiiHome])
						{
							mode = drive;
							led(OFF,ON,OFF,OFF);
						}
						break;
					case task1right:
						led(ON,ON,OFF,ON);
						warhorse_state.task_state = warhorse_state.TASK1RIGHT;
						if (buttons_pushed[wiiPlus])
							state = task2;
						if (buttons_pushed[wiiMinus])
							state = task1left;
						if (buttons_pushed[wiiHome])
						{
							mode = drive;
							led(OFF,OFF,ON,OFF);
						}
						break;
					case task2:
						led(ON,ON,ON,OFF);
						warhorse_state.task_state = warhorse_state.TASK2;
						if (buttons_pushed[wiiPlus])
							state = task3;
						if (buttons_pushed[wiiMinus])
							state = task1right;
						if (buttons_pushed[wiiHome])
						{
							mode = drive;
							led(OFF,OFF,OFF,ON);
						}
						break;
					case task3:
						led(OFF,OFF,ON,ON);
						warhorse_state.task_state = warhorse_state.TASK3;
						if (buttons_pushed[wiiPlus])
							state = manual_drive;
						if (buttons_pushed[wiiMinus])
							state = task2;
						if (buttons_pushed[wiiHome])
						{
							mode = drive;
							led(ON,ON,OFF,OFF);
						}
						break;
					default:
						break;
				}
				break;
			case drive:
				warhorse_state.drive_state = warhorse_state.DRIVE;
				if (buttons_pushed[wiiHome])
				{
					mode = menu;
					warhorse_state.drive_state = warhorse_state.STOP;
				}
				if (buttons_pushed[wiiOne])
				{
					mode = paused;
				}
				break;
			case stop:

				break;
			case paused:
				warhorse_state.drive_state = warhorse_state.PAUSED;
				if (buttons_pushed[wiiOne])
				{
					mode = drive;
				}
				break;
			default:
				break;
		}

		state_pub.publish(warhorse_state);

		loop_rate.sleep();
	}
}


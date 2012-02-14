/*
 * manual_move.cpp
 *
 *  Created on: 21 Aug 2011
 *      Author: Morten Ege Jensen, <ege_morten@hotmail.com>
 *  Description: 
	This is a ROS port of the Kbhit-code written by Kjeld Jensen (kbhit.c, kbhit.h, main.c)
 */

#include <ros/ros.h>
#include <string.h>
#include "fmMsgs/kbhit.h"

extern "C"{
	#include "kbhit.h"
}
/* defines */

#define false		0
#define true		1
#define MAX_ANGLE 	360
#define MIN_ANGLE 	-360
#define KEY_SPACE 	32

class Kbhit{
 private:
	char c, esc_key, second_key;
	// Variables
	int speed, angle, max_speed, min_speed, turn_constant, ramp, v_left, v_right;
	// ROS specific
	fmMsgs::kbhit msg;
	ros::Publisher move_pub;
	// Functions
	int check_value(int chechk, int old);
	void robot_drive(int d_spd, int d_ang);
	void keyHandle(char key);
	void stop_robot();

 public:
	int stop;
	void readKeys();
	// Constructors
	Kbhit();
	virtual ~Kbhit();
};	

Kbhit::Kbhit(){
	ROS_INFO("Kbhit onstructor");
	// Set up ROS specific 
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string kbhit_move_topic;
	n.param<std::string>("topic", kbhit_move_topic, "kbhit_topic");
	n.param<int>("max_speed", max_speed, 100);
	n.param<int>("min_speed", min_speed, -100);
	n.param<int>("turn_constant", turn_constant, 3);
	n.param<int>("ramp", ramp, 20);

	move_pub = nh.advertise<fmMsgs::kbhit>(kbhit_move_topic.c_str(), 1000);

	// Initialize variables
	speed = 0;
	angle = 0;
	v_left = 0;
	v_right = 0;

	/* kbhit init*/
	stop = false;
	esc_key = false;
	second_key = false;
	
	/* initialize kbhit */
	kbhit_init();
}

Kbhit::~Kbhit(){
	/* reset kbhit */
	kbhit_done();
}

int Kbhit::check_value(int check, int old){
	int return_value = check;
	if (check > max_speed){
		return_value = max_speed;
	}else if (check > old + ramp){
		return_value = old + ramp;
	}else if (check < min_speed){
		return_value = min_speed;
	}else if (check < old - ramp){
		return_value = old - ramp;
	}
	return return_value;
}

void Kbhit::robot_drive(int d_spd, int d_ang){
	// establish average speed
	speed = speed + d_spd;
	// If no angle occurred, vehicle has to go straight
	if (d_ang != 0){
		angle = angle + d_ang;
	}else{
		angle = 0;
	}

	//Calculate speed for each wheel
	v_left = check_value(speed + turn_constant*(angle)/10, v_left);
	v_right = check_value(speed - turn_constant*(angle)/10, v_right);
	// calculate the speed of the vehicle
	speed = (v_left + v_right)/2;
	// Post the data to ROS
	msg.header.stamp = ros::Time::now();
	msg.left_velocity = v_left;
	msg.right_velocity = v_right;
	msg.velocity = speed;
	msg.angle = angle;
	move_pub.publish(msg);
}

void Kbhit::keyHandle(char key){
	// Theese could be input as a ROS parameter as 'step_size'
	if (key == 'w'){
		robot_drive(5,0);
	}else if(key =='s'){
		robot_drive(-5,0);
	}else if(key =='a'){
		robot_drive(0,-5);
	}else if(key =='d'){
		robot_drive(0,5);
	}
}

void Kbhit::stop_robot(){
	// Stop the vehicle
	v_left = 0;
	v_right = 0;
	speed = 0;
	angle = 0;
	// Post the STOP data to ROS
	msg.header.stamp = ros::Time::now();
	msg.left_velocity = v_left;
	msg.right_velocity = 0;
	msg.velocity = speed;
	msg.angle = angle;
	move_pub.publish(msg);
}

void Kbhit::readKeys(){
	/* test if a key has been pressed */
	if (kbhit_test())
	{
		/* retrieve the pressed key */
		c = kbhit_getchar();

		/* print the numerical value of the key */
		if (c != KEY_ESCAPE && c != KEY_SECOND && !second_key)
			//printf ("Key code: %d\n", c);
			ROS_INFO("Key code: %d\n", c);
		else if (second_key)
			//printf ("Secondary key code: %d\n", c);
			ROS_INFO ("Secondary key code: %d\n", c);

		/* echo numeric keys */
		if (second_key == false && c >= '0' && c <= '9')
			//printf ("Numeric key %c pressed\n", c);
			ROS_INFO ("Numeric key %c pressed\n", c);

		/* handle keys */
		switch (c)
		{
			case KEY_ESCAPE:
				esc_key = true;
				break;
			case KEY_SECOND:
				if (esc_key == true)
					second_key = true;
				break;
			case KEY_ARROW_RIGHT:
				if (second_key == true)
				{
					second_key = false;
					esc_key = false;
					keyHandle('d');
					//printf ("Right arrow pressed\n");
					ROS_INFO ("Right arrow pressed\n");
				}
				break;
			case KEY_ARROW_LEFT:
				if (second_key == true)
				{
					second_key = false;
					esc_key = false;
					keyHandle('a');
					//printf ("Left arrow pressed\n");
					ROS_INFO ("Left arrow pressed\n");
				}
				break;
			case KEY_ARROW_UP:
				if (second_key == true)
				{
					second_key = false;
					esc_key = false;
					keyHandle('w');
					//printf ("Up arrow pressed\n");
					ROS_INFO ("Up arrow pressed\n");
				}
				break;
			case KEY_ARROW_DOWN:
				if (second_key == true)
				{
					second_key = false;
					esc_key = false;
					keyHandle('s');
					//printf ("Down arrow pressed\n");
					ROS_INFO ("Down arrow pressed\n");
				}
				break;
			case KEY_SPACE:
				// Stop robot
				stop_robot();
				break;

			case 'q':
			case 'Q':
				stop = true;
				break;
		}
	}
}

int main(int argc, char **argv){
	// Initialize ROS
	ros::init(argc, argv, "kbhit_move_node");
	// Instantiate Kbhit object
	Kbhit kbhit;
	// Test for keys
	while(ros::ok() && !kbhit.stop){
		kbhit.readKeys();
		ros::spinOnce();
	}
	return 0;
}

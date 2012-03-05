/*************************************************************************************
 # Copyright (c) 2011, Kent Stark Olsen
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 # 1. Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. All advertising materials mentioning features or use of this software
 #    must display the following acknowledgement:
 #    This product includes software developed by the University of Southern Denmark.
 # 4. Neither the name of the University of Southern Denmark nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY KENT STARK OLSEN ''AS IS'' AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL KENT STARK OLSEN BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************
 # File:    hilde_node.cpp
 # Author:  Kent Stark Olsen <kent.stark.olsen@gmail.com>
 # Created: Jun 24, 2011 Kent Stark Olsen
 **************************************************************************************
 # Features:
 #  Controller for the robot Hilde
 *************************************************************************************/

#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/TwistStamped.h"
#include "fmMsgs/serial.h"
//#include "joy/Joy.h"
#include "boost/circular_buffer.hpp"

enum WiimoteButtons
{
	BTN_1, BTN_2, BTN_A, BTN_TRIGGER, BTN_PLUS, BTN_MINUS, BTN_LEFT, BTN_RIGHT, BTN_UP, BTN_DOWN, BTN_HOME
};

class HildeController
{
public:
	//  Constructor
	HildeController();

private:
	//  Node handling
	ros::NodeHandle global_n;
	ros::NodeHandle local_n;

	//	Subscribers/Publishers
	ros::Subscriber twist_subscriber;
	ros::Subscriber serial_subscriber;
	ros::Subscriber wiimote_subscriber;
	ros::Publisher serial_publisher;
	ros::Publisher serial_test_publisher;

	//	Serial messages
	fmMsgs::serial serial_msg_tx;

	//  Twist message
	geometry_msgs::TwistStamped twist_msg;

	//	Parameters
	std::string twist_subscriber_topic;
	std::string serial_subscriber_topic;
	std::string wiimote_subscriber_topic;
	std::string serial_publisher_topic;
	std::string serial_test_publisher_topic;
	double max_linear_velocity, max_angular_velocity;
	double base_link_radius_to_wheels;
	double base_link_length_to_rear_wheel;

	//	Tentative parameter
	double bias_linear_velocity_percentage;

	//	Variables for serial communication
	std::string input_serial_buffer;
	char output_protocol[4];

	//	Variables for input
	double angular_velocity_z, linear_velocity_x;

	//	Variables for output
	double output_motor_left, output_motor_right, output_servo_angle;

	//	Variables
	int counter;
	bool direction;
	int callback_counter;
	double left_motor_velocity, right_motor_velocity, rear_wheel_angle, temp_rear_wheel_angle, actual_left_velocity, actual_right_velocity, actual_angular_velocity;
	boost::circular_buffer<double> left_encoder_time;
	boost::circular_buffer<double> right_encoder_time;
	boost::circular_buffer<double> left_encoder_ticks;
	boost::circular_buffer<double> right_encoder_ticks;

	//  Callback methods
	void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_data);
	void serialCallback(const fmMsgs::serial::ConstPtr& serial_data);
	//void wiimoteCallback(const joy::Joy::ConstPtr& wiimote_data);

	//	Methods
	void sendSerialCommand(char type, char command, char value, bool publish_to_test_topic);
	void calculateMotorPWM();
};

HildeController::HildeController()
{
	global_n = ros::NodeHandle();
	local_n = ros::NodeHandle("~");

	//  Get parameters from parameter server if it's avaiable, otherwise set to default values.
	local_n.param<std::string>("twist_subscriber_topic", twist_subscriber_topic, "wii_cmd_vel");
	local_n.param<std::string>("serial_subscriber_topic", serial_subscriber_topic, "serial_rx");
	local_n.param<std::string>("wiimote_subscriber_topic", wiimote_subscriber_topic, "joy");
	local_n.param<std::string>("serial_publisher_topic", serial_publisher_topic, "serial_tx");
	local_n.param<std::string>("serial_test_publisher_topic", serial_test_publisher_topic, "serial_test_tx");
	local_n.param("max_linear_velocity", max_linear_velocity, 1.0);
	local_n.param("max_angular_velocity", max_angular_velocity, 1.0);
	local_n.param("bias_linear_velocity_percentage", bias_linear_velocity_percentage, 0.8);
	local_n.param("base_link_radius_to_wheels", base_link_radius_to_wheels, 0.185);
	local_n.param("base_link_length_to_rear_wheel", base_link_length_to_rear_wheel, 0.28);

	//	Subscribers and publisher
	twist_subscriber = global_n.subscribe<geometry_msgs::TwistStamped>(twist_subscriber_topic, 100, &HildeController::twistCallback, this);
	serial_subscriber = global_n.subscribe<fmMsgs::serial>(serial_subscriber_topic, 100, &HildeController::serialCallback, this);
	//wiimote_subscriber = global_n.subscribe<joy::Joy>(wiimote_subscriber_topic, 100, &HildeController::wiimoteCallback, this);
	serial_publisher = global_n.advertise<fmMsgs::serial>(serial_publisher_topic, 100);
	serial_test_publisher = global_n.advertise<fmMsgs::serial>(serial_test_publisher_topic, 100);

	//	Instantiate buffers
	left_encoder_ticks = boost::circular_buffer<double>(10);
	right_encoder_ticks = boost::circular_buffer<double>(10);
	left_encoder_time = boost::circular_buffer<double>(10);
	right_encoder_time = boost::circular_buffer<double>(10);

	//	Initialize values
	counter = 0;
	direction = true;
	callback_counter = 0;
	angular_velocity_z = 0;
	linear_velocity_x = 0;
	output_motor_left = 0;
	output_motor_right = 0;
	output_servo_angle = 0;

	//	Initialize buffers
	for (int i = 0; i <= 9; i++)
	{
		left_encoder_ticks.push_back(0.0);
		right_encoder_ticks.push_back(0.0);
		left_encoder_time.push_back(0.0);
		right_encoder_time.push_back(0.0);
	}
}

//	Callback methods
void HildeController::serialCallback(const fmMsgs::serial::ConstPtr& serial_data)
{
	//	Handle incomming data on serial port
	input_serial_buffer = (serial_data -> data.c_str());

	if (input_serial_buffer[0] == (char)0xDD)
	{
		if (input_serial_buffer[1] == (char)0x60)
		{
			//ROS_INFO("Yeah!!! Left Encoder");

			left_encoder_ticks.push_back((double)input_serial_buffer[2]);
			left_encoder_time.push_back((double)ros::Time::now().toSec());

			actual_left_velocity = 0;

			for (int i = 0; i <= 4; i++)
			{
				actual_left_velocity += left_encoder_ticks[i];
			}

			actual_left_velocity = actual_left_velocity * 0.52 / (left_encoder_time[0] - left_encoder_time[4]);

			//actual_left_velocity = left_encoder_ticks[0];
		}
		if (input_serial_buffer[1] == (char)0x70)
		{
			//ROS_INFO("Yeah!!! Right Encoder");

			right_encoder_ticks.push_back((double)input_serial_buffer[2]);
			right_encoder_time.push_back((double)ros::Time::now().toSec());

			for (int i = 0; i <= 4; i++)
			{
				actual_right_velocity += right_encoder_ticks[i];
			}

			actual_right_velocity = actual_right_velocity * 0.52 / (right_encoder_time[0] - right_encoder_time[4]);

			//actual_right_velocity = right_encoder_ticks[0];
		}
	}

	actual_angular_velocity = (actual_right_velocity - actual_left_velocity) / (2 * base_link_radius_to_wheels);

	//ROS_INFO("LV: %f RV: %f, AV: %f", actual_left_velocity, actual_right_velocity, actual_angular_velocity);
	ROS_INFO("%s", serial_data -> data.c_str());
}

void HildeController::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_data)
{
	angular_velocity_z = (twist_data -> twist.angular.z);
	linear_velocity_x = (twist_data -> twist.linear.x);
	ROS_INFO("twistCallback: %f %f", linear_velocity_x, angular_velocity_z);
	calculateMotorPWM();
}

//void HildeController::wiimoteCallback(const joy::Joy::ConstPtr& wiimote_data)
//{
//
//	/*if (callback_counter++ > 9)
//	{
//		callback_counter = 0;
//
//		if (wiimote_data -> buttons[BTN_TRIGGER])
//		{
//			calculateMotorPWM();
//		}
//		else
//		{
//			output_motor_left 	= 0;
//			output_motor_right 	= 0;
//			output_servo_angle  = 128;
//		}
//
//		sendSerialCommand((char)0xAA, (char)0x10, (char)output_motor_left, true);
//		sendSerialCommand((char)0xAA, (char)0x30, (char)output_motor_right, true);
//		sendSerialCommand((char)0xAA, (char)0x50, (char)output_servo_angle, true);
//
//		//ROS_INFO("L: %f, R: %f, V: %f", output_motor_left, output_motor_right, output_servo_angle);
//	}*/
//}

//	Methods
void HildeController::sendSerialCommand(char type, char command, char value, bool publish_to_test_topic)
{
	//	Setup message
	output_protocol[0] = type;
	output_protocol[1] = command;
	output_protocol[2] = value;
	if (output_protocol[2] == 0) output_protocol[2] = 1;
	output_protocol[3] = '\0';

	//	Send message
	serial_msg_tx.data = output_protocol;
	serial_publisher.publish(serial_msg_tx);
	if (publish_to_test_topic) serial_test_publisher.publish(serial_msg_tx);
}

void HildeController::calculateMotorPWM()
{
	counter++;

	//	Calculate rear wheel angle
	// Angular_velocity_z should be less than 1
//	angular_velocity_z = angular_velocity_z / 20;
	temp_rear_wheel_angle = (atan2(angular_velocity_z * base_link_length_to_rear_wheel, linear_velocity_x) * 180) / M_PI;
	ROS_INFO("rear_wheel: %f", temp_rear_wheel_angle);
	if (temp_rear_wheel_angle < -90) temp_rear_wheel_angle = - 180 -temp_rear_wheel_angle;
	if (temp_rear_wheel_angle > 90) temp_rear_wheel_angle = 180 - temp_rear_wheel_angle;

	if (temp_rear_wheel_angle >= -60.0 && temp_rear_wheel_angle <= 60)
	{
		rear_wheel_angle = -temp_rear_wheel_angle;
		left_motor_velocity = linear_velocity_x + (angular_velocity_z * base_link_radius_to_wheels);
		right_motor_velocity = linear_velocity_x - (angular_velocity_z * base_link_radius_to_wheels);
	}

	//ROS_INFO("L: %f, R: %f, V: %f, TV: %f", left_motor_velocity, right_motor_velocity, rear_wheel_angle, temp_rear_wheel_angle);

	/*
	if (linear_velocity_x >= 0)
	{
		if (!direction)
		{
			direction = true;
			sendSerialCommand((char)0xAA, (char)0x0F, (char)1, true); //	setDirection in firmware will handle this
		}
	}
	else
	{
		if (direction)
		{
			direction = false;
			sendSerialCommand((char)0xAA, (char)0x0F, (char)2, true); //	setDirection in firmware will handle this
		}
	}
	*/

	output_motor_left  = 255.0 * fabs(left_motor_velocity) * bias_linear_velocity_percentage;
	output_motor_right = 255.0 * fabs(right_motor_velocity) * bias_linear_velocity_percentage;
	output_servo_angle = 128 + rear_wheel_angle;

	if (counter >= 0) /// Was 20
	{
		counter = 0;
		ROS_INFO("Sending: %d %d %d", (unsigned char)output_motor_left, (unsigned char)output_motor_right, (unsigned char)output_servo_angle);
		ROS_INFO("Sending: %f %f %f", output_motor_left, output_motor_right, output_servo_angle);
		sendSerialCommand((char)0xAA, (char)0x10, (char)output_motor_left, true);
		sendSerialCommand((char)0xAA, (char)0x30, (char)output_motor_right, true);
		sendSerialCommand((char)0xAA, (char)0x50, (char)output_servo_angle, true);
	}
}

int main(int argc, char **argv)
{
    //  Initialize ROS
    ros::init(argc, argv, "hilde_controller");

    //  Make instance of object
    HildeController hilde_controller = HildeController();

    ROS_WARN("Hilde_controller is alive!");

    //  Loop
    ros::spin();
}

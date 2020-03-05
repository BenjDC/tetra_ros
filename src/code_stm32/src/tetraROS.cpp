/*
 * tetraROS.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "stm32f4xx_hal.h"

#include "main.h"
#include "motor.h"
#include "tetraROS.h"
#include "encoder.h"
#include "pid.h"
#include "battery.h"
#include "mymath.h"
#include <string.h>
#include <stdio.h>


ros::NodeHandle nh;
std_msgs::String str_msg;
geometry_msgs::Twist cmd_vel;

std_msgs::Float32 x_pos_msg;
std_msgs::Float32 y_pos_msg;
std_msgs::Float32 th_pos_msg;
std_msgs::Float32 lin_speed_msg;
std_msgs::Float32 ang_speed_msg;



HAL_Encoder_HandleTypeDef hencoder;


double lin_speed_scaled;
double ang_speed_scaled;

double x_pos;
double y_pos;
double ang_pos;
double x_speed;
double y_speed;
uint16_t last_time_us;



double xpid_kp = 4000.0; // 1000.0
double xpid_ki = 100.0; // 2.0
double xpid_kd = 0.0;
pid_win_handler lin_pid = {
		&xpid_kp,
		&xpid_ki,
		&xpid_kd,
	    -1000.0f,
		1000.0f,
	    100,
	    0.5f
};
double wpid_kp = 20.0;
double wpid_ki = 0.4;
double wpid_kd = 0.0;
pid_win_handler ang_pid = {
		&wpid_kp,
		&wpid_ki,
		&wpid_kd,
	    -1000.0f,
		1000.0f,
	    100,
	    0.5f
};

// callback on command velocity reception
void cmd_vel_cb(const geometry_msgs::Twist& motor_command);


char hello[] = "Hello world!";
float compact_odom[3];//linear_speed, angular speed, timestamp.

//nav_msgs::Odometry odom;

ros::Publisher chatter("chatter", &str_msg);
//ros::Publisher odom_pub("odom", &odom);

//ros::Publisher compact_odom_pub("compact_odom", &c_odom);

ros::Publisher x_pos_pub("compact_odom", &x_pos_msg);
ros::Publisher y_pos_pub("compact_odom", &y_pos_msg);
ros::Publisher th_pos_pub("compact_odom", &th_pos_msg);
ros::Publisher lin_speed_pub("compact_odom", &lin_speed_msg);
ros::Publisher ang_speed_pub("compact_odom", &ang_speed_msg);


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);
//tf::TransformBroadcaster odom_broadcaster;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}


void cmd_vel_cb(const geometry_msgs::Twist& motor_command)
{

	double xspeed_command = motor_command.linear.x;
	double wspeeed_command = motor_command.angular.z;

	//debug line
	//if ((xspeed_command != 0) && (wspeeed_command != 0))
	{
		lin_speed_scaled = xspeed_command * xspeed_max / joy_max;
		ang_speed_scaled = wspeeed_command * wspeed_max / joy_max;
	}

}


void printHello()
{
	str_msg.data = hello;
	chatter.publish(&str_msg);
}


void tellBatteryLevel()
{
	float power_level;
	float power_level_unit;

	power_level = HAL_Battery_Get(VBATT);
	power_level_unit = (float)power_level/3.0;


	// Allocates storage
	char *battery_msg = (char*)malloc(40 * sizeof(char));

	sprintf(battery_msg, "Battery voltage : %f\n", power_level_unit);

	nh.loginfo("TetraROS initialization OK\n");
	nh.loginfo(battery_msg);

}


void initHardware()
{
	HAL_Motor_Init();
	HAL_Encoder_Init(&hencoder,&htim5,&htim2);
	HAL_Battery_Init();

	HAL_TIM_Base_Start(&htim14);
	last_time_us = __HAL_TIM_GET_COUNTER(&htim14);

}
void initTetraROS()
{
	initHardware();

	HAL_Delay(1000);
	HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_AUTO,0);

	nh.initNode();

	nh.subscribe(cmd_vel_sub);
	nh.advertise(chatter);

/*
    c_odom.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    c_odom.layout.dim[0].label = "lidar";
    c_odom.layout.dim[0].size = 3;
    c_odom.layout.dim[0].stride = 1*3;
    c_odom.layout.data_offset = 0;
    c_odom.layout.dim_length = 1;
    c_odom.data_length = 3;
*/

	//nh.advertise(compact_odom_pub);
	nh.advertise(x_pos_pub);
	nh.advertise(y_pos_pub);
	nh.advertise(th_pos_pub);
	nh.advertise(lin_speed_pub);
	nh.advertise(ang_speed_pub);


	x_pos = 0;
	y_pos = 0;
	ang_pos =0;


	tellBatteryLevel();
}





void loopTetraROS()
{

	nh.spinOnce();
	//ros::Time currentTime = nh.now();
	uint16_t current_time_us = __HAL_TIM_GET_COUNTER(&htim14);
	uint16_t delta_time_us = current_time_us-last_time_us;


	if(delta_time_us>=4808) //! 208Hz (ODR)
	{

		last_time_us = current_time_us;
		float period_us = (float)(delta_time_us)/1000000.0f; //s
		printHello();

		// Attitude PID motor controler
		float lin_speed_actual = 0.0f;
		float ang_speed_actual = 0.0f;
		float lin_pwm = 0.0f;
		float ang_pwm = 0.0f;
		{
		  int32_t const left_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_LEFT);
		  int32_t const right_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_RIGHT);

		  float const lin_delta_count = (float)(left_count+right_count)/2.0f;
		  static double const distance_per_pulse = (PI_FLOAT*0.144f)/4480.0f; // 0.1mm per pulse , 4480 pulses per revolution, 0.45m per revolution
		  float const lin_distance = (float)(lin_delta_count)*distance_per_pulse; // m
		  lin_speed_actual = lin_distance/period_us;

		  float const ang_delta_count = (float)(right_count-left_count);
		  float const ang_distance = (float)(ang_delta_count)*distance_per_pulse; // m
		  float const ang_degrees = ToDeg(fastAtan2(ang_distance,0.30)); // degree, wheel base = 30cm
		  ang_speed_actual = ang_degrees/period_us;

		  float const lin_error = lin_speed_scaled-lin_speed_actual;
		  float const ang_error = ang_speed_scaled-ang_speed_actual;

		  lin_pwm = process_pid_win(&lin_pid,lin_error);
		  ang_pwm = process_pid_win(&ang_pid,ang_error);

		  HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,(int32_t)(lin_pwm+ang_pwm));
		  HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,(int32_t)(lin_pwm-ang_pwm));



		  // odometry

/*
		  compact_odom[0] = lin_speed_actual;
		  compact_odom[1] = ang_speed_actual;
		  compact_odom[2] = (float)current_time_us;

		  c_odom.data = compact_odom;
*/
		  //compact_odom_pub.publish(&c_odom);

		  ang_pos += ang_degrees;
		  x_pos += lin_distance * cos(ang_pos);
		  y_pos += lin_distance * sin(ang_pos);
		  x_speed = lin_speed_actual * cos(ang_pos);
		  y_speed = lin_speed_actual * sin(ang_pos);

		  x_pos_msg.data = x_pos;
		  y_pos_msg.data = y_pos;
		  th_pos_msg.data = ang_pos;

		  lin_speed_msg.data = lin_speed_actual;
		  ang_speed_msg.data = ang_speed_actual;

		  x_pos_pub.publish(&x_pos_msg);
		  y_pos_pub.publish(&y_pos_msg);
		  th_pos_pub.publish(&th_pos_msg);

		  lin_speed_pub.publish(&lin_speed_msg);
		  ang_speed_pub.publish(&ang_speed_msg);
		  /*

		  ros::Time currentTime = nh.now();

			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(ang_pos);

			//first, we'll publish the transform over tf

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = currentTime;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x_pos;
			odom_trans.transform.translation.y = y_pos;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);



			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = currentTime;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x_pos;
			odom.pose.pose.position.y = y_pos;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = x_speed;
			odom.twist.twist.linear.y = y_speed;
			odom.twist.twist.angular.z = ang_speed_actual;

			//publish the message
			odom_pub.publish(&odom);
*/


		}
	}

	//printHello();





}




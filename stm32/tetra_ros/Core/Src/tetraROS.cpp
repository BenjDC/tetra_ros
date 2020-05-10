/*
 * tetraROS.cpp

 *
 *  Created on: 2020/01/17
 *      Author: BenjDC
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <tetra_ros/compactOdom.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "main.h"
#include "motor.h"
#include "tetraROS.h"
#include "encoder.h"
#include "pid.h"
#include "led.h"
#include "battery.h"
#include "mymath.h"



ros::NodeHandle nh;
geometry_msgs::Twist msg_cmd_vel;
tetra_ros::compactOdom msg_compact_odom;
std_msgs::Int16 msg_command_management;

HAL_Encoder_HandleTypeDef hencoder;
HAL_LED_HandleTypeDef hled;
//HAL_LED_HandleTypeDef hled;

double lin_speed_scaled;
double ang_speed_scaled;

float x_pos;
float y_pos;
float ang_pos;
int total_right_count;
int total_left_count;
uint32_t last_time_us;
uint32_t last_vel_cmd;
bool connected;

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
void cmd_mgmt_cb(const std_msgs::Int16& management_command);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);
ros::Subscriber<std_msgs::Int16> cmd_mgmt_sub("cmd_mgmt", &cmd_mgmt_cb);
ros::Publisher compactOdom_pub("compact_odom", &msg_compact_odom);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void cmd_mgmt_cb(const std_msgs::Int16& management_command)
{
	int mgmt_command = management_command.data;

	switch(mgmt_command)
	{
	case RESET_ODOMETRY:
		nh.loginfo("Resetting odometry");
		x_pos =0;
		y_pos = 0;
		ang_pos =0;
		total_right_count = 0;
		total_left_count = 0;
		break;
	case SWITCH_MODE:
		nh.loginfo("switch mode (not implemented yet)\n");
		break;
	case GET_BATTERY_STATE:
		tellBatteryLevel();
		break;
	default:
		break;

	}
}

void cmd_vel_cb(const geometry_msgs::Twist& motor_command)
{

    double xspeed_command = motor_command.linear.x;
    double wspeeed_command = motor_command.angular.z;

    lin_speed_scaled = xspeed_command * xspeed_max / joy_max;
    ang_speed_scaled = wspeeed_command * wspeed_max / joy_max;

    last_vel_cmd = __HAL_TIM_GET_COUNTER(&htim14);
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

    nh.loginfo(battery_msg);

    free(battery_msg);
}


void initHardware()
{
	// init motor
    HAL_Motor_Init();

    // init encoder
    HAL_Encoder_Init(&hencoder,&htim5,&htim2);

    // init battery
    HAL_Battery_Init();

    // init timer
    HAL_TIM_Base_Start(&htim14);
    last_time_us = __HAL_TIM_GET_COUNTER(&htim14);

	//init led
    HAL_Led_Init();
    HAL_Led_Add(&hled,LED_GPIO_Port,LED_Pin);


}
void initTetraROS()
{
    initHardware();

    HAL_Delay(1000);
    HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_AUTO,0);
    
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(cmd_mgmt_sub);
    nh.advertise(compactOdom_pub);

    x_pos = 0;
    y_pos = 0;
    ang_pos =0;
    connected=false;
}



void loopTetraROS()
{

    nh.spinOnce();

    uint32_t current_time_us = __HAL_TIM_GET_COUNTER(&htim14);
    uint32_t delta_time_us = current_time_us-last_time_us;



    if(delta_time_us>=4808) //! 208Hz (ODR)
    {
		if (!nh.connected())
		{
            //cut motors 
            HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_AUTO,0);
			lin_speed_scaled = 0;
			ang_speed_scaled = 0;
			connected = false;
			HAL_Led_Reset(&hled);
		}
		else if (!connected)
		{
			connected = true;
			nh.loginfo("TetraROS STM32 connected\n");
		    tellBatteryLevel();
			HAL_Led_Set(&hled);
		}


        last_time_us = current_time_us;
        float period_us = (float)(delta_time_us)/1000000.0f; //s

        // Attitude PID motor controler
        float lin_speed_actual = 0.0f;
        float ang_speed_actual = 0.0f;
        float lin_pwm = 0.0f;
        float ang_pwm = 0.0f;
        
		int32_t const left_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_LEFT);
		int32_t const right_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_RIGHT);

		float const lin_delta_count = (float)(left_count+right_count)/2.0f;
		static double const distance_per_pulse = (PI_FLOAT*0.144f)/4480.0f; // 0.1mm per pulse , 4480 pulses per revolution, 0.45m per revolution
		float const lin_distance = (float)(lin_delta_count)*distance_per_pulse; // m
		lin_speed_actual = lin_distance/period_us;

		float const ang_delta_count = (float)(right_count-left_count);
		float const ang_distance = (float)(ang_delta_count)*distance_per_pulse; // m
		float const ang_degrees = ToDeg(fastAtan2(ang_distance,0.29)); // degree, wheel base = 30cm
		ang_speed_actual = ang_degrees/period_us;

		float const lin_error = lin_speed_scaled-lin_speed_actual;
		float const ang_error = ang_speed_scaled-ang_speed_actual;

		lin_pwm = process_pid_win(&lin_pid,lin_error);
		ang_pwm = process_pid_win(&ang_pid,ang_error);

		// send pwm to motors
		HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,(int32_t)(lin_pwm+ang_pwm));
		HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,(int32_t)(lin_pwm-ang_pwm));

		// odometry
		ang_pos += ang_degrees;
		x_pos += lin_distance * cos(ToRad(ang_pos));
		y_pos += lin_distance * sin(ToRad(ang_pos));
		total_right_count += right_count;
		total_left_count += left_count;

		msg_compact_odom.x_pos = x_pos;
		msg_compact_odom.y_pos = y_pos;
		msg_compact_odom.ang_pos = ang_pos;
		msg_compact_odom.x_speed = lin_speed_actual * cos(ang_pos);
		msg_compact_odom.y_speed = lin_speed_actual * sin(ang_pos);
		msg_compact_odom.ang_speed = ang_speed_actual;
		msg_compact_odom.stamp = nh.now();

		compactOdom_pub.publish(&msg_compact_odom);
    }
}




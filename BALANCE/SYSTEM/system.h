#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
#include "FreeRTOSConfig.h"
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "show.h"								   
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"
#include "HubDriver.h"
#include "HubDriverCan.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "filter.h"

#include "stmflash.h"

//Is it an Ackermann car, or a differential car
//是阿克曼结构小车，还是差速小车
#define Akm_Car   0
#define Diff_Car  1

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder_metre; //Encoder value, motor real-time speed, unit: mm //编码器数值，电机实时速度，单位：mm/s
	float Encoder_Rpm;   //Encoder value, motor real-time speed, unit: 0.1rpm //编码器数值，电机实时转速，单位：0.1rpm
	float Control_Rpm;   //Control the target speed of the brushless motor, unit: 0.1rpm //控制无刷电机的目标转速，单位：0.1rpm
	float Control_metre;   //Control the target speed of the brushless motor, unit: mm/s //控制无刷电机的目标速度，单位：mm/s
	float Motor_Pwm; //the pid value for control  speed of the brushless motor, unit: 0.1rpm//pid输出的值，代表转速，单位：0.1rpm
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop,Last_Flag_Stop;
extern int Divisor_Mode;   
extern u8 Car_Mode;
extern float RC_Velocity, Turn_rate;
extern float Move_X,Move_Y,Move_Z;
extern float Balance_Kp, Balance_Kd; 
extern float Balance_V_KP,Balance_V_KI;
extern float Velocity_KP,Velocity_KI;
extern float Angle_Zero;
extern Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter; 
extern float Wheel_spacing; 
extern float Wheel_axlespacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart_ON_Flag;   
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send; 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int robot_mode_check_flag;

void systemInit(void);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY	100
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值
#define CAR_NUMBER    10 
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif

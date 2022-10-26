#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"
#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

extern int rpm_max;
extern int Encoder_Integral_Max;
extern int Time1_count, Time3_count, Time8_count;

#define FILTERING_TIMES 20

float Balance(float Angle,float Gyro);
float Balance_V(int encoder_left, int encoder_right);
float Move_X_control(float Move_X, float encoder_left, float encoder_right);
	
void Balance_task(void *pvParameters);
float Limit_data(float data, int max);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
float float_abs(float insert);
void Get_RC(void);
void PS2_control(void);
void Remote_Control(void);
float Incremental_PI_Move (float Encoder,float Target);
float Incremental_PI_Turn (float Encoder,float Target);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void robot_mode_check(void);
void Smooth_control(float vx, float step);
int Smooth_steering(int currentPWM, int targetPWM, float step);
int Mean_Filter(int data);
int Mean_Filter_gyro(int data);
void MPU6050_EXTI_Init(void);
#define INT PBin(15)   //连接到MPU6050的中断引脚

#endif  


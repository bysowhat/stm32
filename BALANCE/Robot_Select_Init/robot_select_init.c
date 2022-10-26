#include "robot_select_init.h"

//Initialize the robot parameter structure
//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	Robot_Init(HUB_wheelspacing,  0, 0, 0, HUB_200_Diameter); 
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 电机减速比 电机编码器精度 轮胎直径 
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
  Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing 轮距  
	Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing 轴距
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio //电机减速比
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines //编码器精度(编码器线数)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel //主动轮轮径
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing 轮距 
  Wheel_spacing=Robot_Parament.WheelSpacing;     
}

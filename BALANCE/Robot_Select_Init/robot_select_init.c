#include "robot_select_init.h"

//Initialize the robot parameter structure
//��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef  Robot_Parament;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	Robot_Init(HUB_wheelspacing,  0, 0, 0, HUB_200_Diameter); 
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
�������ܣ���ʼ��С������
��ڲ������־� ��� ������ٱ� ������������� ��ֱ̥�� 
����  ֵ����
**************************************************************************/
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
  Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing �־�  
	Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing ���
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio //������ٱ�
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines //����������(����������)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel //�������־�
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//���(����)ת1Ȧ��Ӧ�ı�������ֵ
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference //�������ܳ�	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing �־� 
  Wheel_spacing=Robot_Parament.WheelSpacing;     
}

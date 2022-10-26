/**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

//Control speed pins
//����ת������
#define PWMA   TIM8->CCR2 
#define PWMB   TIM8->CCR1 

//Control positive and negative retractor pins
//��������ת����
#define INA1   PCout(12)  
#define INB1   PDout(2)   
#define INA2   PBout(5)   
#define INB2   PBout(4)    

//Pins that determine whether the motor is allowed to move
//�жϵ���Ƿ������˶�������
#define EN     PAin(12) 

//Steering gear control pin
//�����������
#define SERVO  TIM4->CCR2
//Actuator zero macro definition
//������궨��
#define SERVO_INIT 1500     

void Enable_Pin(void);
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
void Servo_PWM_Init(u16 arr,u16 psc);
#endif

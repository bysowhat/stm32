#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //�������ȼ�
#define LED_STK_SIZE 		128   //Task stack size //�����ջ��С

#define LED PBout(13) //LED Pin //LED����

void LED_Init(void);  
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif
#include "led.h"

int Led_Count; //LED flicker time control //LED��˸ʱ�����

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
�������ܣ�LED�ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//Enable port clock //ʹ�ܶ˿�ʱ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //Port configuration //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//Push-pull output //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);	//Initialize GPIO with the specified parameters //�����趨������ʼ��GPIO
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
�������ܣ�LED����˸����
��ڲ������� 
����  ֵ����
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED״̬ȡ����0�ǵ�����1��Ϩ��    
      LED=~LED;   

      if(Flag_Stop) 
				Led_Count=10;	
			else
				Led_Count=300;
			
      //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
      //LED��˸����ǳ��򵥣���Ƶ�ʾ���Ҫ��ͣ�ʹ�������ʱ����			
      vTaskDelay(Led_Count); 
    }
}  

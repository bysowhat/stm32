#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable port clock //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	//Port configuration //�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	
  //Pull up input //��������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	//Initialize GPIOD with the set parameters //�����趨������ʼ��GPIOD
  GPIO_Init(GPIOB, &GPIO_InitStructure);					
} 
/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
	//Press the release sign
	//�������ɿ���־
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed //��������
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed //�ް�������
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������3s
**************************************************************************/
u8 Long_Press(void)
{
	static int PressCount;
	int Pressed,PressTimeCount=10;
	if(KEY==0&&PressCount<(PressTimeCount+1)) PressCount++;
	if(KEY==1)
	{
		Delay_ms();
		if(KEY==1)PressCount=0,Pressed=0;
	}
	if((PressTimeCount)==PressCount) Pressed=1;
	
	if(Pressed==1) 
	{
		Pressed=0;
		return 1;
	}
	else return 0;
}
/**************************************************************************
Function: Delay function
Input   : none
Output  : none
�������ܣ��ӳٺ���
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<500;i++);
	 }	
}

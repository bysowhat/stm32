#include "motor.h"

/**************************************************************************
Function: Motor orientation pin initialization
Input   : none
Output  : none
�������ܣ�����������ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable port clock //ʹ�ܶ˿�ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
	
	//Port configuration //�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;
	//Push-pull output //�������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_12;
	//Push-pull output //�������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	//Push-pull output //�������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//IO output 0to prevent motor transfer
  //IO���0����ֹ�����ת
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	GPIO_ResetBits(GPIOC,GPIO_Pin_4);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	GPIO_ResetBits(GPIOC,GPIO_Pin_12);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
}
/**************************************************************************
Function: The motor PWM initialization
Input   : psc: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ����PWM���ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//Enable timer 8 //ʹ�ܶ�ʱ��8 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  //Enable GPIO peripheral clock //ʹ��GPIO����ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
	//Reuse push-pull output //����������� 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	//Up counting mode 
	//���ϼ���ģʽ  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  
	//Channel preload enable
	//ͨ��Ԥװ��ʹ��
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  
	
  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	//Channel preload enable
	//ͨ��Ԥװ��ʹ��
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

  // Enable the TIMX preloaded register on the ARR
  //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	TIM_ARRPreloadConfig(TIM8, ENABLE); 
	
	//Enable TIM8
	//ʹ��TIM8
	TIM_Cmd(TIM8, ENABLE);  
	
	// Advanced timer output must be enabled
	//�߼���ʱ���������ʹ�����		
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 		
} 

/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
�������ܣ�ʹ�ܿ������ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //Enable port clock  //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;            //Port configuration //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //Pull up input      //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //Initialize GPIO with the specified parameters //�����趨������ʼ��GPIO
} 
/**************************************************************************
Function: Steering gear PWM initialization
Input   : Auto-Reload Register, Prescaler
Output  : none
�������ܣ����PWM��ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{	 
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//Enable timer 4 //ʹ�ܶ�ʱ��4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//Enable GPIO peripheral clock //ʹ��GPIO����ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	//Port configuration //�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
	//Reuse push-pull output //����������� 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	//Initialize GPIO with the specified parameters   //�����趨������ʼ��GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	//Up counting mode 
	//���ϼ���ģʽ  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	//Channel preload enable
	//ͨ��Ԥװ��ʹ��	 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
 
  //Enable the TIMX preloaded register on the ARR
  //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ��� 
	TIM_ARRPreloadConfig(TIM4, ENABLE); 
	
	//Enable TIM4
	//ʹ��TIM4
	TIM_Cmd(TIM4, ENABLE);  
	
	//The channel value is initialized to 1500, corresponding to the steering gear zero
	//ͨ��ֵ��ʼ��Ϊ1500���������Ӧֵ
	TIM4->CCR2=1500;	
}

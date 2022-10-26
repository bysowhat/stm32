/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V3.5
�޸�ʱ�䣺2021-01-29

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V3.5
Update��2021-01-29

All rights reserved
***********************************************/

#include "system.h"

//Robot software fails to flag bits
//���������ʧ�ܱ�־λ
u8 Flag_Stop=1, Last_Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺ�
int Divisor_Mode;

// Robot type variable
//�������ͺű���
u8 Car_Mode=0; 

//Vehicle three-axis target moving speed, unit: m/s
//Note that Move_Y does not work and only makes sense for omnidirectional moving cars
//С������Ŀ���˶��ٶȣ���λ��m/s
//ע��Move_Y�������ã�ֻ�ж�ȫ���ƶ�С��Move_Y��������
float Move_X, Move_Y, Move_Z;   

//Default speed of remote control car, unit: mm/s
//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=1500; 
float Turn_rate=1;

//PID parameters of balance
//ƽ�����PID����
float Balance_Kp=2.00,  Balance_Kd=1.00;
float Balance_V_KP=5.00, Balance_V_KI=1.00; 
float Velocity_KP=5.36, Velocity_KI=0.52; 
float Angle_Zero=50;
int gyro_0, rpm_max=150;
int Encoder_Integral_Max=10000; 

//The parameter structure of the motor
//����Ĳ����ṹ��
Motor_parameter MOTOR_A, MOTOR_B;  

/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//����������
float Encoder_precision; 
//Wheel circumference, unit: m
//�����ܳ�����λ��m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//�������־࣬��λ��m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//С��ǰ�������࣬��λ��m
float Wheel_axlespacing; 

/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1 communication control flag bit.
//These 5 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2�ֱ�������APP����ģ�ֱ���CANͨ�š�����1ͨ�ſ��Ʊ�־λ����5����־λĬ�϶�Ϊ0��������3����ģʽ
u8 PS2_ON_Flag=0, APP_ON_Flag=1, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart_ON_Flag=0; 

//Bluetooth remote control associated flag bits
//����ң����صı�־λ
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//������APP���Ͳ����ı�־λ
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2�ֱ�������ر���
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

void systemInit(void)
{
	//Turn off the JTAG interface to enable the OLED display
	//�ر�JTAG�ӿڣ��ر�JTAG�ӿڲ��ܿ���OLED��ʾ��
	JTAG_Set(JTAG_SWD_DISABLE); 
	
	//Open the SWD interface debug interface
	//����SWD�ӿڵ��Խӿ�
	JTAG_Set(SWD_ENABLE);           
	
	//Interrupt priority group setting
	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//Delay function initialization
	//��ʱ������ʼ��
	delay_init();			                                            
	
	//Initialize the hardware interface connected to the LED lamp
	//��ʼ����LED�����ӵ�Ӳ���ӿ�
	LED_Init();   
	
	//Initialize the hardware interface connected to the enable switch
	//��ʼ����ʹ�ܿ������ӵ�Ӳ���ӿ�
	Enable_Pin();

  //Initialize the hardware interface connected to the OLED display
  //��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�	
	OLED_Init();     
	
	//Initialize the hardware interface connected to the user's key
	//��ʼ�����û��������ӵ�Ӳ���ӿ�
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//����1��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart1_init(115200);	  
	
	//Serial port 2 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//����2��ʼ����ͨ�Ų�����9600������������APP��ͨ��
	uart2_init(9600);  
	
	//Serial port 3 is initialized and the baud rate is 115200. 
	//Serial port 3 is the default port used to communicate with ROS terminal
	//����3��ʼ����ͨ�Ų�����115200������3ΪĬ��������ROS��ͨ�ŵĴ���
	uart3_init(115200);

  //Initialize the CAN communication interface
  //CANͨ�Žӿڳ�ʼ��
	CAN1_Mode_Init(1,2,3,12,0); 
	
	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC���ų�ʼ�������ڶ�ȡ��ص�ѹ���λ����λ����λ����λ����С���������С�������ͺ�
 	Adc_Init();  
  			
  //IIC��ʼ��
	IIC_Init();      
  //MPU6050��ʼ��		
  MPU6050_initialize();     
  //��ʼ��DMP  	
	DMP_Init();                                
			
	//According to the tap position of the potentiometer, determine which type of car needs to be matched, 
  //and then initialize the corresponding parameters	
  //���ݵ�λ���ĵ�λ�ж���Ҫ���������һ���ͺŵ�С����Ȼ����ж�Ӧ�Ĳ�����ʼ��	
  Robot_Select();
	
	//���������ʼ��
	HUB_init();    

	//Initialize the model aircraft remote control interface. 
	//Note that the pins of serial port 1 and model aircraft remote control conflict. 
	//When opened at the same time, serial port 1 will send data wrong
	//��ʼ����ģң�ؽӿڣ�Ҫע����Ǵ���1�ͺ�ģң�ص����ų�ͻ��ͬʱ����ʱ������1�������ݻ����
  TIM1_Cap_Init(0XFFFF,72-1); 
	
  //Initialize the hardware interface to the PS2 controller
	//��ʼ����PS2�ֱ����ӵ�Ӳ���ӿ�
	PS2_Init();
	//PS2 gamepad configuration is initialized and configured in analog mode
  //PS2�ֱ����ó�ʼ��,����Ϊģ����ģʽ	
	PS2_SetInit(); 
   	
	//200HZ�����Ŀ��ƽ���
	//�ⲿ�жϣ���MPU6050���ж����Ŵ���
	MPU6050_EXTI_Init();
}

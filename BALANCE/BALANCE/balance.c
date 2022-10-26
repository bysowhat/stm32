#include "balance.h"

//Time variable //��ʱ����
int Time1_count=0, Time3_count=0, Time8_count=0;

//С��ƽ���ȶ�λ�ÿ�����ر���
float Encoder_Integral; 

/**************************************************************************
Function: Trolley balance control
Input   : Angle, angular velocity
Output  : Vehicle target speed
�������ܣ�С��ƽ�����
��ڲ������Ƕȣ����ٶ�(ǰ��Ϊ��)
����  ֵ��С��Ŀ���ٶ�(ǰ��Ϊ��)
**************************************************************************/
int Gyro_polarity=-1;
float Balance(float Angle,float Gyro)
{  
   float Bias;
	 float balance;
	 Bias=Angle-Angle_Zero/100;                     //���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
/**************************************************************************
Function: Trolley balance and stability control
Input   : Left wheel speed, right wheel speed
Output  : Vehicle target speed
�������ܣ�С��ƽ���ȶ�����
��ڲ������Ƕȣ����ٶ�(ǰ��Ϊ��)
����  ֵ��С��Ŀ���ٶ�(ǰ��Ϊ��)
**************************************************************************/
float Balance_V(int encoder_left, int encoder_right)
{  
    static float Velocity,Encoder_new,Encoder;

   //=============�ٶ�PI������=======================//	
		Encoder_new =(encoder_left+encoder_right)-0; //������Ҳ��ϲ����ᵼ�����ҷֱ������������ҷ����෴
		Encoder *= 0.8;		                                                                    //һ�׵�ͨ�˲���       
		Encoder += Encoder_new*0.2;	                                                          //һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                                           //���ֳ�λ��
		Encoder_Integral=Encoder_Integral;                                                    //����ң�������ݣ�����ǰ������
		if(Encoder_Integral> Encoder_Integral_Max)  Encoder_Integral= Encoder_Integral_Max;   //�����޷�
		if(Encoder_Integral<-Encoder_Integral_Max)	Encoder_Integral=-Encoder_Integral_Max;   //�����޷�	
		Velocity=Encoder*Balance_V_KP/100+Encoder_Integral*Balance_V_KI/1000;                 //�ٶȿ���	
	
		if(Flag_Stop==1)   Encoder_Integral=0;      //����رպ��������
	
	  return Velocity;
}
/**************************************************************************
Function: Balance the trolley core control process, Frequency of 100 hz
Input   : none
Output  : none
�������ܣ�ƽ��С�����Ŀ��ƽ��̣�Ƶ��200hz
��ڲ�������
����  ֵ����
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{ 
	if(INT==0)
	{
		EXTI->PR=1<<12;   //����жϱ�־λ   
		
		/*�����˶�����*/
		Read_DMP();	 //��ȡ��ǡ����ٶ�����
		
		//if(Pitch>39 || Pitch<-39 || 
		//	 MOTOR_A.Encoder_Rpm>rpm_max*12 || MOTOR_A.Encoder_Rpm<-rpm_max*12 ||
		 //  MOTOR_B.Encoder_Rpm>rpm_max*12 || MOTOR_B.Encoder_Rpm<-rpm_max*12)
		//{
		// Flag_Stop=1;
		//}

		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0, or the model detection marker is 0
		//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0�������ͺż���־λΪ0
		if(Turn_Off(Voltage)==0)
		{
			//����APP��PS2�ֱ�����ģ�ֱ�����ƽ�⳵
			//������ΪMove_X��Move_Z��ע������Ŀ�������û�е�λ�ģ�ֻ�����ٶȵĴ�С�������޷���ȷָ��ƽ�⳵��ǰ����ת���ٶ�
			//if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //����APPң������
			//else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //����ģң������
			//else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //����PS2�ֱ���������

			//ƽ�⻷
			//MOTOR_A.Control_Rpm  = Balance(Pitch, gyro[0]);   
			//MOTOR_B.Control_Rpm  = Balance(Pitch, gyro[0]);  
	
			//�ٶȻ�
			//MOTOR_A.Control_Rpm  = MOTOR_A.Control_Rpm + Balance_V(MOTOR_A.Encoder_Rpm, MOTOR_B.Encoder_Rpm)-Move_X*10-Move_Z*5; 
			//MOTOR_B.Control_Rpm  = MOTOR_B.Control_Rpm + Balance_V(MOTOR_A.Encoder_Rpm, MOTOR_B.Encoder_Rpm)-Move_X*10+Move_Z*5; 
			
			
			// moving speed of car(unit m/s), this speed is recieved from uart3, debugonly
			//С�����徶������ٶ� m/s ,���ٶ��Ǵ���3��ros���յ���,�ô�������ڵ���
			//3.6����ÿСʱ���൱���˵Ĳ����ٶȵ�1/3
			float CAR_target_x = 0.33;
			// rotation speed of car(unit rad/s), this speed is recieved from uart3, debugonly
			//С��������ת�ٶ� 0.rad/s�����ٶ��Ǵ���3��ros���յ���,�ô�������ڵ���
			// 5����ת90��
			// if CAR_target_z>0, ��motor bת��motor a������������ת
			// if CAR_target_z>0, turn right to left
			float CAR_target_z = 0.314;
			
			//�������ֲ����˶�ѧ��ʽ����С�������ٶȣ�ת��Ϊ��������������ٶ�
		  //Inverse kinematics //�˶�ѧ���
			MOTOR_A.Control_metre = CAR_target_x * 1000 - CAR_target_z * Wheel_spacing*1000 / 2.0f; 
			MOTOR_B.Control_metre = CAR_target_x *1000 + CAR_target_z * Wheel_spacing*1000 / 2.0f; 
			
			//convert mm/s to rps
			//��Ŀ���ٶ�ת��ΪĿ��ת��
			Get_Target_Encoder_Form_Velocity();
			
			
			//�ٶȿ��ƣ�ͨ�����ƾ����ٶȺ������ٶȣ��ﵽǰ����ת���ԭ��ת��
			//Speed closed-loop control to calculate the PWM value of each motor, 
		  //PWM represents the actual wheel speed					 
		  //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��
		  MOTOR_A.Motor_Pwm=Incremental_PI_Move(MOTOR_A.Encoder_Rpm, MOTOR_A.Control_Rpm);
		  MOTOR_B.Motor_Pwm=Incremental_PI_Move(MOTOR_B.Encoder_Rpm, MOTOR_B.Control_Rpm);
				 
			
		
			//�ٶ��޷�
			MOTOR_A.Motor_Pwm = Limit_data(MOTOR_A.Motor_Pwm, rpm_max);
			MOTOR_B.Motor_Pwm = Limit_data(MOTOR_B.Motor_Pwm, rpm_max);
			
			//���Ƶ��
			//hub_CAN_Syn_Rpm 
			//input0:motor a, left, the gps side round per minus
			//input1:motor b, right, round per minus
			hub_CAN_Syn_Rpm(MOTOR_A.Motor_Pwm/10, -MOTOR_B.Motor_Pwm/10);
			//hub_CAN_Syn_Rpm(MOTOR_A.Control_Rpm/10, -MOTOR_B.Control_Rpm/10);
			//hub_CAN_Syn_Rpm(60, 0);
		}
		
		//��ͣ
		if(Flag_Stop==1 && Last_Flag_Stop==0)CAN_Stop();
		if(Flag_Stop==0 && Last_Flag_Stop==1)	
		{
			CAN_acc_time(acc_time);
      CAN_Enable();
		}
		Last_Flag_Stop=Flag_Stop;
		/*�����˶�����*/
		
		//��ȡ���Ͻǵ�λ��������ƽ��������
		Angle_Zero=(Get_Adc(Angle_Zero_ADC)-2048)/5;
		
		//��ȡ��ص�ѹ
		Voltage_All+=Get_battery_volt();                
		if(++Voltage_Count==10) Voltage=Voltage_All/10,Voltage_All=0,Voltage_Count=0;
		
		//OLED��ʾ����ʾ
		oled_show();
		
		//��APP��������
		//APP_Show();	 
		
		//ͨ������3���ͻ�����״̬����
		data_transition(); 
		USART3_SEND();
		
		//��ȡPS2�ֱ���������
		//PS2_Read(); 	

		//Click the user button to update the gyroscope zero
		//�����û������������������
		Key();
		
	}
	return 0;
}
/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float Limit_data(float data, int max)
{	
	if(data> max)data= max;
	if(data<-max)data=-max;
	
	return data;
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
		u8 temp;
		if(voltage<20||EN==0||Flag_Stop==1)
		{	                                                
			temp=1;      
			PWMA=0;
			PWMB=0;					
		}
		else
			temp=0;
		return temp;			
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
float Incremental_PI_Move (float Encoder,float Target)
{
	 float KP=1, KI=0.1;
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=KP*(Bias-Last_bias)+KI*Bias; 
	 if(Pwm>50)Pwm=20;
	 if(Pwm<-50)Pwm=-20;//debug_flag
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
float Incremental_PI_Turn (float Encoder,float Target)
{
	 float KP=10, KI=1;
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=KP*(Bias-Last_bias)+KI*Bias; 
	 if(Pwm>50)Pwm=10;
	 if(Pwm<-50)Pwm=-10;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	 switch(Flag_Direction) //Handle direction control commands //�������������
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	

	if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
	Move_Z=Move_Z*RC_Velocity/500;	
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
}
/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
   	float LX,LY,RY;
		int Yuzhi=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);
		LX=-(PS2_LY-128);
		RY=-(PS2_RX-128);
	  //RX=-(PS2_RY-128);

	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	  if     (PS2_KEY==11) RC_Velocity+=1;  //To accelerate//����
		else if(PS2_KEY==9)	 RC_Velocity-=1;  //To slow down //����
	
		if(RC_Velocity<0)    RC_Velocity=0;

		Move_X=LX;
		Move_Z=RY;
	  Move_X=Move_X*RC_Velocity/128;
		Move_Z=RY*PI/2/128;
		
		if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;

	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
}
/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=100;
    int Yuzhi=100; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
	
	  //limiter //�޷�
    int LX,LY,RY; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

		//Front and back direction of left rocker. Control forward and backward.
	  //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=Remoter_Ch2-1500;
	  //The channel is not currently in use
	  //��ͨ����ʱû��ʹ�õ�
    LY=Remoter_Ch4-1500;
	  //Right stick left and right. To control the rotation. 
		//��ҡ�����ҷ��򡣿�����ת��
    RY=-(Remoter_Ch1-1500);//��ת

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
			
		//The remote control command of model aircraft is processed
		//�Ժ�ģң�ؿ���������д���
		Move_X=LX;
		Move_Z=RY;
		Move_X=Move_X*RC_Velocity/500;
		Move_Z=RY*PI/2/500;
		
		if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;	
			
		//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	  //���ٳ�Z����˳(<0)��(>0)ʱ����ת
		//The greater the forward speed, the greater the rotation speed
	  //ǰ���ٶ�Խ����ת�ٶ�Խ��
		if(Move_X<0)Move_Z=-Move_Z;

	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
		
		//Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;

}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click(); 
	if(tmp==1)
	{
		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));
		Deviation_Pitch=Pitch;
		Flag_Stop=!Flag_Stop;
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
  //The encoder converts the raw data to wheel speed in m/s
	//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
	MOTOR_A.Encoder_metre = MOTOR_A.Encoder_Rpm/60*Wheel_perimeter/10; //����10����Ϊ��ȡ����ת�ٵ�λΪ0.1rpm
	MOTOR_B.Encoder_metre = MOTOR_B.Encoder_Rpm/60*Wheel_perimeter/10;
}
/**************************************************************************
Function: Read the target wheel speed( unit m/s) and calculate target encoder valu( unit 0.1rpm)
Input   : none
Output  : none
�������ܣ�ͨ��Ŀ�공���ٶȣ���λmm/s������Ŀ�������rpm����λ��0.1rpm
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Target_Encoder_Form_Velocity(void)
{
	MOTOR_A.Control_Rpm = MOTOR_A.Control_metre/(Wheel_perimeter*1000)*10*60; //����10����Ϊת�ٵ�λΪ0.1rpm
	MOTOR_B.Control_Rpm = MOTOR_B.Control_metre/(Wheel_perimeter*1000)*10*60;
}
/**************************************************************************
Function: Smoothing the front wheel steering speed to prevent excessive steering gear current
Input   : Current servo PWM, Target servo PWM, Smooth value
Output  : none
�������ܣ���ǰ��ת���ٶ���ƽ��������ֹ�����������
��ڲ�������ǰ�������PWMֵ Ŀ��������PWMֵ ƽ��ֵ
����  ֵ����
**************************************************************************/
int Smooth_steering(int currentPWM, int targetPWM, float step)
{
	int threshold=7;
	if     (targetPWM>currentPWM+threshold) currentPWM+=step;
	else if(targetPWM<currentPWM-threshold) currentPWM-=step;
	else                                    currentPWM =targetPWM;
  
	return currentPWM;
}
/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
�������ܣ����ݻ����˲�
��ڲ���������
����  ֵ���˲��������
**************************************************************************/
int Mean_Filter(int data)
{
  u8 i;
  s32 Sum_Speed = 0; 
  s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =data;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
  return Filter_Speed;
}
int Mean_Filter_gyro(int data)
{
  u8 i;
  s32 Sum_Speed = 0; 
  s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =data;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
  return Filter_Speed;
}

/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void MPU6050_EXTI_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIO�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	            //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource15);
	EXTI_InitStructure.EXTI_Line=EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//��ռ���ȼ�2�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 
}


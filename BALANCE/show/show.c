#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern float Slide_target;
  
/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
extern int Encoder_Integral;
void oled_show(void)
{
	static int gyro_show;

	if(gyro[0]<0 && -gyro[0]>float_abs(gyro_show))gyro_show=gyro[0];
	if(gyro[0]>0 &&  gyro[0]>float_abs(gyro_show))gyro_show=gyro[0];

	Voltage_Show=Voltage*100;
			
	{
	 //The first line of the display displays the content//
	 //��ʾ����1����ʾ����//	
	OLED_ShowString(0,0,"ZERO");
	 if( Angle_Zero<0)  OLED_ShowString(35,0,"-"),OLED_ShowNumber(45,0,-Angle_Zero,3,12);
	 else               OLED_ShowString(35,0,"+"),OLED_ShowNumber(45,0, Angle_Zero,3,12);   
		
	 OLED_ShowString(68,0,"PITC"); //Zero-drift data of Z axis gyroscope //z�����������Ư������
	 if(Pitch<0)   OLED_ShowString(103,0,"-"),OLED_ShowNumber(110,0,-Pitch,3,12);
	 else          OLED_ShowString(103,0,"+"),OLED_ShowNumber(110,0, Pitch,3,12);

	 //The second line of the display displays the content//
	 //��ʾ����2����ʾ����//	
	 //Display Z-axis angular velocity //��ʾZ����ٶ�
	 OLED_ShowString(00,10,"GYRO-P"); 
	 if( gyro[0]<0)   OLED_ShowString(60,10,"-"),
										OLED_ShowNumber(75,10,-gyro[0],5,12);
	 else             OLED_ShowString(60,10,"+"),
										OLED_ShowNumber(75,10, gyro[0],5,12);	

	 //The third line of the display displays the content//
	 //��ʾ����3����ʾ����//		
	 //Display the target speed and current speed of motor A
	 //��ʾ���A��Ŀ���ٶȺ͵�ǰ�ٶ�
	 OLED_ShowString(0,20,"L:");
	 if( MOTOR_A.Control_Rpm<0)	OLED_ShowString(15,20,"-"),
													OLED_ShowNumber(20,20,-MOTOR_A.Control_Rpm/60*Wheel_perimeter*1000,5,12);
	 else                 	OLED_ShowString(15,20,"+"),
													OLED_ShowNumber(20,20, MOTOR_A.Control_Rpm/60*Wheel_perimeter*1000,5,12); 
		
	 if( MOTOR_A.Encoder_metre<0)	OLED_ShowString(60,20,"-"),
													OLED_ShowNumber(75,20,-MOTOR_A.Encoder_metre*1000,5,12);
	 else                 	OLED_ShowString(60,20,"+"),
													OLED_ShowNumber(75,20, MOTOR_A.Encoder_metre*1000,5,12);

	 //The fourth line of the display displays the content//
	 //��ʾ����4����ʾ����//
	 //Display the target speed and current speed of motor B
	 //��ʾ���B��Ŀ���ٶȺ͵�ǰ�ٶ�
	 OLED_ShowString(0,30,"R:");
	 if( MOTOR_B.Control_Rpm<0)	OLED_ShowString(15,30,"-"),
													OLED_ShowNumber(20,30,- MOTOR_B.Control_Rpm/60*Wheel_perimeter*1000,5,12);
	 else                 	OLED_ShowString(15,30,"+"),
													OLED_ShowNumber(20,30,  MOTOR_B.Control_Rpm/60*Wheel_perimeter*1000,5,12); 
			
	 if( MOTOR_B.Encoder_metre<0)	OLED_ShowString(60,30,"-"),
													OLED_ShowNumber(75,30,-MOTOR_B.Encoder_metre*1000,5,12);
   else                 	OLED_ShowString(60,30,"+"),
													OLED_ShowNumber(75,30, MOTOR_B.Encoder_metre*1000,5,12);

	//Line 5 of the display displays the content//
	//��ʾ����5����ʾ����//
	//Displays the current battery voltage
  //��ʾ��ǰ��ص�ѹ	
	 OLED_ShowString(0,40,"Voltage: ");
														OLED_ShowString(88,40,".");
														OLED_ShowString(110,40,"V");
														OLED_ShowNumber(75,40,Voltage_Show/100,2,12);
														OLED_ShowNumber(98,40,Voltage_Show%100,2,12);
	if(Voltage_Show%100<10) 	OLED_ShowNumber(92,40,0,2,12);
	}


	//��ʾ����6����ʾ����
	//Line 6 of the display displays the contents
	//Displays the current control mode //��ʾ��ǰ����ģʽ
	if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2   ");
	else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP   ");
	else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C   ");
	else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN   ");
	else if (Usart_ON_Flag==1) OLED_ShowString(0,50,"USART1");
	else                       OLED_ShowString(0,50,"USART3");

	//Displays whether controls are allowed in the current car
	//��ʾ��ǰС���Ƿ��������	
	if(EN==1&&Flag_Stop==0)   OLED_ShowString(85,50,"O N"); 
	else                      OLED_ShowString(85,50,"OFF");
	
	//Refresh the screen //ˢ����Ļ
	OLED_Refresh_Gram();		
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //�Ե�ص�ѹ����ɰٷֱ���ʽ
	 Voltage_Show=(Voltage*100-2000)*5/26;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	 Left_Figure=MOTOR_A.Encoder_metre/10;  if(Left_Figure<0)Left_Figure=-Left_Figure;	 
	 Right_Figure=MOTOR_B.Encoder_metre/10; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //���ڽ����ӡAPP���ݺ���ʾ����
	 flag_show=!flag_show;
	
	 if(PID_Send==1)
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //���Ͳ�����APP��APP�ڵ��Խ�����ʾ
		 printf("{C%d:%d:%d:%d:%d:%d}$",
		 (int)RC_Velocity,
		 (int)(Balance_Kp*100),
		 (int)(Balance_Kd*100),
		 (int)(Balance_V_KP*100),
		 (int)(Balance_V_KI*100),
		 (int) Encoder_Integral_Max
//		 (int) Angle_Zero+1000
			 );
		 
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0)
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //���Ͳ�����APP��APP����ҳ��ʾ
	   printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[0]); 
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //���Ͳ�����APP��APP�ڲ��ν�����ʾ
	   printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }
}



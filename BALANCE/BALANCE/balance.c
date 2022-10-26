#include "balance.h"

//Time variable //计时变量
int Time1_count=0, Time3_count=0, Time8_count=0;

//小车平衡稳定位置控制相关变量
float Encoder_Integral; 

/**************************************************************************
Function: Trolley balance control
Input   : Angle, angular velocity
Output  : Vehicle target speed
函数功能：小车平衡控制
入口参数：角度，角速度(前倾为正)
返回  值：小车目标速度(前进为正)
**************************************************************************/
int Gyro_polarity=-1;
float Balance(float Angle,float Gyro)
{  
   float Bias;
	 float balance;
	 Bias=Angle-Angle_Zero/100;                     //求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
/**************************************************************************
Function: Trolley balance and stability control
Input   : Left wheel speed, right wheel speed
Output  : Vehicle target speed
函数功能：小车平衡稳定控制
入口参数：角度，角速度(前倾为正)
返回  值：小车目标速度(前进为正)
**************************************************************************/
float Balance_V(int encoder_left, int encoder_right)
{  
    static float Velocity,Encoder_new,Encoder;

   //=============速度PI控制器=======================//	
		Encoder_new =(encoder_left+encoder_right)-0; //如果左右不合并，会导致左右分别反馈，导致左右方向相反
		Encoder *= 0.8;		                                                                    //一阶低通滤波器       
		Encoder += Encoder_new*0.2;	                                                          //一阶低通滤波器    
		Encoder_Integral +=Encoder;                                                           //积分出位移
		Encoder_Integral=Encoder_Integral;                                                    //接收遥控器数据，控制前进后退
		if(Encoder_Integral> Encoder_Integral_Max)  Encoder_Integral= Encoder_Integral_Max;   //积分限幅
		if(Encoder_Integral<-Encoder_Integral_Max)	Encoder_Integral=-Encoder_Integral_Max;   //积分限幅	
		Velocity=Encoder*Balance_V_KP/100+Encoder_Integral*Balance_V_KI/1000;                 //速度控制	
	
		if(Flag_Stop==1)   Encoder_Integral=0;      //电机关闭后清除积分
	
	  return Velocity;
}
/**************************************************************************
Function: Balance the trolley core control process, Frequency of 100 hz
Input   : none
Output  : none
函数功能：平衡小车核心控制进程，频率200hz
入口参数：无
返回  值：无
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{ 
	if(INT==0)
	{
		EXTI->PR=1<<12;   //清除中断标志位   
		
		/*核心运动控制*/
		Read_DMP();	 //读取倾角、角速度数据
		
		//if(Pitch>39 || Pitch<-39 || 
		//	 MOTOR_A.Encoder_Rpm>rpm_max*12 || MOTOR_A.Encoder_Rpm<-rpm_max*12 ||
		 //  MOTOR_B.Encoder_Rpm>rpm_max*12 || MOTOR_B.Encoder_Rpm<-rpm_max*12)
		//{
		// Flag_Stop=1;
		//}

		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0, or the model detection marker is 0
		//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0，或者型号检测标志位为0
		if(Turn_Off(Voltage)==0)
		{
			//蓝牙APP、PS2手柄、航模手柄控制平衡车
			//控制量为Move_X和Move_Z，注意这里的控制量是没有单位的，只代表速度的大小，但是无法精确指定平衡车的前进、转向速度
			//if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			//else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
			//else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令

			//平衡环
			//MOTOR_A.Control_Rpm  = Balance(Pitch, gyro[0]);   
			//MOTOR_B.Control_Rpm  = Balance(Pitch, gyro[0]);  
	
			//速度环
			//MOTOR_A.Control_Rpm  = MOTOR_A.Control_Rpm + Balance_V(MOTOR_A.Encoder_Rpm, MOTOR_B.Encoder_Rpm)-Move_X*10-Move_Z*5; 
			//MOTOR_B.Control_Rpm  = MOTOR_B.Control_Rpm + Balance_V(MOTOR_A.Encoder_Rpm, MOTOR_B.Encoder_Rpm)-Move_X*10+Move_Z*5; 
			
			
			// moving speed of car(unit m/s), this speed is recieved from uart3, debugonly
			//小车整体径向方向的速度 m/s ,该速度是串口3从ros接收到的,该代码仅用于调试
			//3.6公里每小时，相当于人的步行速度的1/3
			float CAR_target_x = 0.33;
			// rotation speed of car(unit rad/s), this speed is recieved from uart3, debugonly
			//小车整体旋转速度 0.rad/s，该速度是串口3从ros接收到的,该代码仅用于调试
			// 5秒钟转90度
			// if CAR_target_z>0, 从motor b转向motor a，即从右向左转
			// if CAR_target_z>0, turn right to left
			float CAR_target_z = 0.314;
			
			//根据两轮差速运动学公式，将小车整体速度，转换为左右两个电机的速度
		  //Inverse kinematics //运动学逆解
			MOTOR_A.Control_metre = CAR_target_x * 1000 - CAR_target_z * Wheel_spacing*1000 / 2.0f; 
			MOTOR_B.Control_metre = CAR_target_x *1000 + CAR_target_z * Wheel_spacing*1000 / 2.0f; 
			
			//convert mm/s to rps
			//把目标速度转换为目标转速
			Get_Target_Encoder_Form_Velocity();
			
			
			//速度控制，通过控制径向速度和切向速度，达到前进和转弯或原地转弯
			//Speed closed-loop control to calculate the PWM value of each motor, 
		  //PWM represents the actual wheel speed					 
		  //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
		  MOTOR_A.Motor_Pwm=Incremental_PI_Move(MOTOR_A.Encoder_Rpm, MOTOR_A.Control_Rpm);
		  MOTOR_B.Motor_Pwm=Incremental_PI_Move(MOTOR_B.Encoder_Rpm, MOTOR_B.Control_Rpm);
				 
			
		
			//速度限幅
			MOTOR_A.Motor_Pwm = Limit_data(MOTOR_A.Motor_Pwm, rpm_max);
			MOTOR_B.Motor_Pwm = Limit_data(MOTOR_B.Motor_Pwm, rpm_max);
			
			//控制电机
			//hub_CAN_Syn_Rpm 
			//input0:motor a, left, the gps side round per minus
			//input1:motor b, right, round per minus
			hub_CAN_Syn_Rpm(MOTOR_A.Motor_Pwm/10, -MOTOR_B.Motor_Pwm/10);
			//hub_CAN_Syn_Rpm(MOTOR_A.Control_Rpm/10, -MOTOR_B.Control_Rpm/10);
			//hub_CAN_Syn_Rpm(60, 0);
		}
		
		//急停
		if(Flag_Stop==1 && Last_Flag_Stop==0)CAN_Stop();
		if(Flag_Stop==0 && Last_Flag_Stop==1)	
		{
			CAN_acc_time(acc_time);
      CAN_Enable();
		}
		Last_Flag_Stop=Flag_Stop;
		/*核心运动控制*/
		
		//读取左上角电位器，调节平衡倾角零点
		Angle_Zero=(Get_Adc(Angle_Zero_ADC)-2048)/5;
		
		//读取电池电压
		Voltage_All+=Get_battery_volt();                
		if(++Voltage_Count==10) Voltage=Voltage_All/10,Voltage_All=0,Voltage_Count=0;
		
		//OLED显示屏显示
		oled_show();
		
		//向APP发送数据
		//APP_Show();	 
		
		//通过串口3发送机器人状态数据
		data_transition(); 
		USART3_SEND();
		
		//读取PS2手柄控制命令
		//PS2_Read(); 	

		//Click the user button to update the gyroscope zero
		//单击用户按键更新陀螺仪零点
		Key();
		
	}
	return 0;
}
/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
float Incremental_PI_Move (float Encoder,float Target)
{
	 float KP=1, KI=0.1;
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=KP*(Bias-Last_bias)+KI*Bias; 
	 if(Pwm>50)Pwm=20;
	 if(Pwm<-50)Pwm=-20;//debug_flag
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
float Incremental_PI_Turn (float Encoder,float Target)
{
	 float KP=10, KI=1;
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=KP*(Bias-Last_bias)+KI*Bias; 
	 if(Pwm>50)Pwm=10;
	 if(Pwm<-50)Pwm=-10;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
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
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
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
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	

	if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
	Move_Z=Move_Z*RC_Velocity/500;	
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
}
/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	float LX,LY,RY;
		int Yuzhi=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);
		LX=-(PS2_LY-128);
		RY=-(PS2_RX-128);
	  //RX=-(PS2_RY-128);

	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	  if     (PS2_KEY==11) RC_Velocity+=1;  //To accelerate//加速
		else if(PS2_KEY==9)	 RC_Velocity-=1;  //To slow down //减速
	
		if(RC_Velocity<0)    RC_Velocity=0;

		Move_X=LX;
		Move_Z=RY;
	  Move_X=Move_X*RC_Velocity/128;
		Move_Z=RY*PI/2/128;
		
		if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;

	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
}
/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100;
    int Yuzhi=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
	
	  //limiter //限幅
    int LX,LY,RY; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

		//Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500;
	  //The channel is not currently in use
	  //该通道暂时没有使用到
    LY=Remoter_Ch4-1500;
	  //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=-(Remoter_Ch1-1500);//自转

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
			
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
		Move_X=LX;
		Move_Z=RY;
		Move_X=Move_X*RC_Velocity/500;
		Move_Z=RY*PI/2/500;
		
		if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;	
			
		//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	  //差速车Z代表顺(<0)逆(>0)时针旋转
		//The greater the forward speed, the greater the rotation speed
	  //前进速度越大旋转速度越大
		if(Move_X<0)Move_Z=-Move_Z;

	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
		
		//Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;

}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
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
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
  //The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s
	MOTOR_A.Encoder_metre = MOTOR_A.Encoder_Rpm/60*Wheel_perimeter/10; //除以10是因为读取到的转速单位为0.1rpm
	MOTOR_B.Encoder_metre = MOTOR_B.Encoder_Rpm/60*Wheel_perimeter/10;
}
/**************************************************************************
Function: Read the target wheel speed( unit m/s) and calculate target encoder valu( unit 0.1rpm)
Input   : none
Output  : none
函数功能：通过目标车轮速度（单位mm/s）计算目标编码器rpm，单位是0.1rpm
入口参数：无
返回  值：无
**************************************************************************/
void Get_Target_Encoder_Form_Velocity(void)
{
	MOTOR_A.Control_Rpm = MOTOR_A.Control_metre/(Wheel_perimeter*1000)*10*60; //乘以10是因为转速单位为0.1rpm
	MOTOR_B.Control_Rpm = MOTOR_B.Control_metre/(Wheel_perimeter*1000)*10*60;
}
/**************************************************************************
Function: Smoothing the front wheel steering speed to prevent excessive steering gear current
Input   : Current servo PWM, Target servo PWM, Smooth value
Output  : none
函数功能：对前轮转向速度做平滑处理，防止舵机电流过大
入口参数：当前舵机控制PWM值 目标舵机控制PWM值 平滑值
返回  值：无
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
函数功能：数据滑动滤波
入口参数：数据
返回  值：滤波后的数据
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
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/
void MPU6050_EXTI_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIO端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	            //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource15);
	EXTI_InitStructure.EXTI_Line=EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 
}


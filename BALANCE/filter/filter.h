#ifndef __FILTER_H
#define __FILTER_H
#include "system.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
extern float angle, angle_dot; 	
float Kalman_Filter(float Accel,float Gyro);		
float Yijielvbo(float angle_m, float gyro_m);
#endif

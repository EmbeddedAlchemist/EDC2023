//头文件

#ifndef PID_H
#define PID_H

#include "pid.h"
//#include "stm32f10x.h" 

//extern u16 j;



typedef struct PID_Calibration {
        
        float kp; //比例
        float ki; // 积分
        float kd; // 微分
		float actual; // 实际值
        float target; // 目标值
        float time_delta; // 时间
        float previous_error;//过去的误差
        float integral; //累计积分
        float output; // 输出值
		float error;//现在的误差
		float d;//微分的值


    } PID_Calibration ;
   

typedef struct PID_add{
//增量式pid	
	
	
      float kp_add;
	  float ki_add;
	  float kd_add;
	
	  float set_target;
	  float actual;
	  float output;
	
	  float last_error;
	  float error;
	  float next_error;
	
	  
	  
}PID_add;


 void pid_add(PID_add * pid_add , float actu);
 float	pid_iterate(PID_Calibration* pid, float actu);

// end of header 
#endif
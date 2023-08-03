#include "PID.h"
#include "PID.hpp"

//位置式pid
//没有积分限幅
//长时间误差过大会导致难以收敛

float pid_iterate(PID_Calibration* pid, float actu) 
	{
		pid->actual = actu;//实际值
		pid->error =   pid->target - pid->actual;//误差计算
		pid->integral += pid->error;//积分计算
		
		pid->d = (pid->error -pid->previous_error)/pid->time_delta;//微分计算
		pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->d;//pid计算
		pid->previous_error = pid->error;//误差记录
	  /*  if(pid->output <=0)
		{
			pid->output = 0;
		}		
		if(pid->output>=5)
		{
			pid->output = 5.0;
		}
		*/
		return pid->output;
		
	}

    //增量式pid

void pid_add(PID_add * pid_add , float actu)
{
	pid_add->actual = actu;
	
	pid_add->error = pid_add->set_target - pid_add->actual;
	
	pid_add->output += pid_add->kp_add*(pid_add->error - pid_add->next_error) + pid_add->ki_add*pid_add->error + pid_add->kd_add * (pid_add->error - 2* pid_add->next_error + pid_add->last_error);
	/*if(pid_add->output < 0)
	{
		pid_add->output = 0;
	}
	if(pid_add->output >5 )
	{
		pid_add->output = 5.0;
	}*/
	pid_add->last_error = pid_add->next_error;
	
	pid_add->next_error = pid_add->error;
	
}



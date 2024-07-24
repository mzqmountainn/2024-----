#include <PID.h>

/*************************************************************************
*  功能说明：限幅函数
*  参数说明：
* @param    amt   ： 参数
* @param    low   ： 最低值
* @param    high  ： 最高值
*************************************************************************/
float constrain_float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid参数初始化函数
void PidInit(pid_param_t * pid)
{
  pid->kp                = 0;
  pid->ki                = 0;
  pid->kd                = 0;
  pid->imax              = 0;
  pid->out_p             = 0;
  pid->out_i             = 0;
  pid->out_d             = 0;
  pid->out               = 0;
  pid->integrator        = 0;
  pid->last_error        = 0;
  pid->last_derivative   = 0;
  pid->last_t   		 = 0;
}

/*************************************************************************
*  功能说明：pid位置式控制器输出
*  参数说明：
* @param    pid     pid参数
* @param    error   pid输入误差
*************************************************************************/

float PidLocCtrl(pid_param_t * pid, float error)
{
  /* 累积误差 */
  pid->integrator += error;
  
  /* 误差限幅 */
  pid->integrator=constrain_float(pid->integrator, -pid->imax, pid->imax);
  pid->out_p = pid->kp * error;
  pid->out_i = pid->ki * pid->integrator;
  pid->out_d = pid->kd * (error - pid->last_error);
 
  pid->last_error = error;
 
  pid->out = pid->out_p + pid->out_i + pid->out_d;
  
  return pid->out;
}



/*************************************************************************
*  功能说明：pid增量式控制器输出
*  特别说明：此函数由chatgpt4o生成 mzq 20240724
*  参数说明：
* @param    pid     pid参数
* @param    error   pid输入误差
*************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error)
{
    float delta_error;
    float delta_d;
    float output;

    // 计算当前误差的增量
    delta_error = error - pid->last_error;

    // 计算微分项增量
    delta_d = delta_error - pid->last_derivative;

    // 计算PID各项的输出
    pid->out_p = pid->kp * delta_error;
    pid->out_i = pid->ki * error; // 积分项直接取当前误差
    pid->out_d = pid->kd * delta_d;

    // 计算PID输出
    output = pid->out_p + pid->out_i + pid->out_d;

    // 更新last_error和last_derivative
    pid->last_error = error;
    pid->last_derivative = delta_error;

    // 返回PID输出
    return output;
}

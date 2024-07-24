#include <PID.h>

/*************************************************************************
*  ����˵�����޷�����
*  ����˵����
* @param    amt   �� ����
* @param    low   �� ���ֵ
* @param    high  �� ���ֵ
*************************************************************************/
float constrain_float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid������ʼ������
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
*  ����˵����pidλ��ʽ���������
*  ����˵����
* @param    pid     pid����
* @param    error   pid�������
*************************************************************************/

float PidLocCtrl(pid_param_t * pid, float error)
{
  /* �ۻ���� */
  pid->integrator += error;
  
  /* ����޷� */
  pid->integrator=constrain_float(pid->integrator, -pid->imax, pid->imax);
  pid->out_p = pid->kp * error;
  pid->out_i = pid->ki * pid->integrator;
  pid->out_d = pid->kd * (error - pid->last_error);
 
  pid->last_error = error;
 
  pid->out = pid->out_p + pid->out_i + pid->out_d;
  
  return pid->out;
}



/*************************************************************************
*  ����˵����pid����ʽ���������
*  �ر�˵�����˺�����chatgpt4o���� mzq 20240724
*  ����˵����
* @param    pid     pid����
* @param    error   pid�������
*************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error)
{
    float delta_error;
    float delta_d;
    float output;

    // ���㵱ǰ��������
    delta_error = error - pid->last_error;

    // ����΢��������
    delta_d = delta_error - pid->last_derivative;

    // ����PID��������
    pid->out_p = pid->kp * delta_error;
    pid->out_i = pid->ki * error; // ������ֱ��ȡ��ǰ���
    pid->out_d = pid->kd * delta_d;

    // ����PID���
    output = pid->out_p + pid->out_i + pid->out_d;

    // ����last_error��last_derivative
    pid->last_error = error;
    pid->last_derivative = delta_error;

    // ����PID���
    return output;
}

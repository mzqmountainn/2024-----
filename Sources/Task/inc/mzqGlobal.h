
#ifndef MZQGLOBAL_H
#define MZQGLOBAL_H

//小车中使用的长度我们使用厘米作为单位  有误差？
#define CarPerRoundLength 14.444  // 4.6*3.14
#define EncoderPerLength 0.0641955 // 14.444/225
//定时器3用于左轮，定时器4用于右轮
extern float LeftSpeed;
extern float RightSpeed;
//tb6612驱动芯片中使用的定义
#define STBY_6612 P10
#define AIN1_6612 P61
#define AIN2_6612 P63
#define BIN1_6612 P11
#define BIN2_6612 P16

#include "PID.h"
extern pid_param_t pid1;
extern pid_param_t pid2;
extern pid_param_t pidLoc;


#endif

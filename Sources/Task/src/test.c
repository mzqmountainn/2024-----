#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "STC32G_UART.h"
#include "STC32G_Delay.h"
//#include "queue.h"
#include "stdio.h"
#include "Semphr.h"
#include "string.h"
#include "math.h"
#include	"STC32G_PWM.h"
#include "mzqGlobal.h"
extern void TX1_write2buff(uint8_t dat);
extern void TX2_write2buff(uint8_t dat);

float LeftSpeed =0;
float RightSpeed=0;
//目标速度
int targetSpeed = 10; 
extern PWMx_Duty PWMA_Duty;
pid_param_t pid1;
pid_param_t pid2;
pid_param_t pidLoc;
/// @brief 位置式pid返回pwm差值
int pwmDeltaPIDloc = 0;
QueueHandle_t pwmUpdateSignal = NULL;
//串口2接收到的openmv原始数据
char rawAngleFromOPENMV[8] = {0x00/*0x2c,0x12,0x00,0x01,0x01,0x01,0x01,0x01*/};
//存储处理后的数据
int realAngle = 0;
//通过8266获取到的是否运行指令
char runFlagFrom8266 = 1;

extern char *itoa(int num, char *str, int radix);
void motorTEST(void);
char *floatToString(float num, int precision, char *str);

void pidControl(void){
  static int pwmaDuty1Raw, pwmaDuty2Raw;
  //pidLoc.kp = (LeftSpeed + RightSpeed) / 2.0;
  pwmDeltaPIDloc = PidLocCtrl(&pidLoc, realAngle);
  pwmDeltaPIDloc = (int)constrain_float(pwmDeltaPIDloc, -50, 50);
  // PWMA_Duty.PWM1_Duty+=PidIncCtrl(&pid1, (targetSpeed - LeftSpeed )/EncoderPerLength+pwmDeltaPIDloc);
  // PWMA_Duty.PWM2_Duty+=PidIncCtrl(&pid2, (targetSpeed - RightSpeed)/EncoderPerLength-pwmDeltaPIDloc);
  pwmaDuty1Raw+=PidIncCtrl(&pid1, (targetSpeed - LeftSpeed )/EncoderPerLength+pwmDeltaPIDloc);
  pwmaDuty2Raw+=PidIncCtrl(&pid2, (targetSpeed - RightSpeed)/EncoderPerLength-pwmDeltaPIDloc);
  if(pwmaDuty1Raw>=2400)
    pwmaDuty1Raw = 2300;
  if(pwmaDuty2Raw>=2400)
    pwmaDuty2Raw = 2300;
  if(pwmaDuty1Raw<= -2400)
    pwmaDuty1Raw = 2300;
  if(pwmaDuty2Raw<= -2400)
    pwmaDuty2Raw = 2300;
  if(pwmaDuty1Raw>=0){
    AIN1_6612 = 1;
    AIN2_6612 = 0;
    PWMA_Duty.PWM1_Duty = pwmaDuty1Raw;
  }else{
    AIN1_6612 = 0;
    AIN2_6612 = 1;
    PWMA_Duty.PWM1_Duty = 0-pwmaDuty1Raw;
  }
  if(pwmaDuty2Raw>=0){
    BIN1_6612 = 1;
    BIN2_6612 = 0;
    PWMA_Duty.PWM2_Duty = pwmaDuty2Raw;
  }else{
    BIN1_6612 = 0;
    BIN2_6612 = 1;
    PWMA_Duty.PWM2_Duty = 0-pwmaDuty2Raw;
  }
  if(PWMA_Duty.PWM1_Duty>=2400)
    PWMA_Duty.PWM1_Duty = 2300;
  if(PWMA_Duty.PWM2_Duty>=2300)
    PWMA_Duty.PWM2_Duty = 2400;
  xSemaphoreGive(pwmUpdateSignal);
}

//计算小车速度并进行pid控制
void outputSpeed(void *pvParameters){
  int Encoder1count = 0;//左
  int Encoder2count = 0;//右
  char output[15];
  pvParameters = pvParameters;
  while (1)
  {
    T3R = 0;//停止定时器工作
    T4R = 0;

    Encoder1count = (T3H << 8) | T3L;
    //itoa(Encoder1count, output, 10);
    //PrintString1(output);
    T3H = 0;
    T3L = 0;

    
    Encoder2count = (T4H << 8) | T4L;
    //itoa(Encoder1count, output, 10);
    //PrintString1(output);
    T4H = 0;
    T4L = 0;

    LeftSpeed = Encoder1count * EncoderPerLength *10;
    RightSpeed = Encoder2count * EncoderPerLength*10;
    //常规向串口1发送
    // floatToString(LeftSpeed, 6, output);
    // PrintString1(output);
    // floatToString(RightSpeed, 6, output);
    // PrintString1(output);

    //通过esp8266发送
    // floatToString(LeftSpeed, 6, output);
    // PrintString3(output);
    // PrintString3("  ");
    // floatToString(RightSpeed, 6, output);
    // PrintString3(output);
    // PrintString3("\n");

    //PWMA_Duty.PWM1_Duty+=PidIncCtrl(&pid1, (35 - LeftSpeed )/EncoderPerLength);
    //PWMA_Duty.PWM2_Duty+=PidIncCtrl(&pid2, (35 - RightSpeed)/EncoderPerLength);


    T3R = 1;//启动定时器
    T4R = 1;
    if(runFlagFrom8266){
      pidControl();}
    //xSemaphoreGive(pwmUpdateSignal);
    vTaskDelay(100);
  }
  
}
//pid更新
void PWMupdate(void *pvParameters){
  static int duty = 600;
  pvParameters = pvParameters;
  motorTEST();
  while (1)
  {
    xSemaphoreTake(pwmUpdateSignal, portMAX_DELAY);
    //PWMA_Duty.PWM1_Duty = duty;
    //PWMA_Duty.PWM2_Duty = duty;
    UpdatePwm(PWMA, &PWMA_Duty);
    vTaskDelay(50);
    //vTaskDelay(500);
  }
}

void motorTEST(void){
  AIN1_6612 = 1;
  AIN2_6612 = 0;
  STBY_6612 = 1;
  BIN1_6612 = 1;
  BIN2_6612 = 0;
}

//串口2接受openmv信息
void openMVgetAngle(void *pvParameters){
  int i = 0;
  char temp[15];
  pvParameters = pvParameters;
  while (1)
  {
    if(COM2.RX_TimeOut > 0 && --COM2.RX_TimeOut == 0 ){
			
			//1.2 判断收到的数据长度 > 0
			if(COM2.RX_Cnt > 0 ){
				
				//1.3 获取数据 :: 数据装在 RX1_Buffer 数组里面去 拿到之后直接发给PC。
        // for(i = 0 ; i < COM2.RX_Cnt  ; i++){
				// 	TX1_write2buff(RX2_Buffer[i]);
				// }
				RX2_Buffer[COM2.RX_Cnt] = '\0';
        //PrintString1(RX2_Buffer);
				//strcpy(rawAngleFromOPENMV,RX2_Buffer);  !!!问题
        //PrintString1(RX2_Buffer);
        for ( i = 0; i < COM2.RX_Cnt; i++)
        {
          temp[i] = RX2_Buffer[i];
        }
        //PrintString1(temp);
        if(temp[0]==0x2c /*&& temp[1] == 0x12 &&temp[5] == 0x5b&&temp[6] == 0xdd*/){
          realAngle = temp[3];
          if(temp[2] == 2){ //车头朝左2 朝右1
            realAngle = 0 - realAngle;
          }
          // itoa(realAngle, temp, 10);
          // PrintString1(temp);
        }
			}
			COM2.RX_Cnt  = 0 ;
      
		}

    vTaskDelay(10);
  }
  
}

//串口3接受esp8266信息
void moudle8266(void *pvParameters){
  static char IfFirstIn = 1;
  int i = 0;
  char temp[15];
  pvParameters = pvParameters;
  while (1)
  {
    if(IfFirstIn){
      IfFirstIn = 0;
      vTaskDelay(3000);
    }
    if(COM3.RX_TimeOut > 0 && --COM3.RX_TimeOut == 0 ){
			
			//1.2 判断收到的数据长度 > 0
			if(COM3.RX_Cnt > 0 ){
				
				//1.3 获取数据 :: 数据装在 RX1_Buffer 数组里面去 拿到之后直接发给PC。
        // for(i = 0 ; i < COM2.RX_Cnt  ; i++){
				// 	TX1_write2buff(RX2_Buffer[i]);
				// }
				RX3_Buffer[COM3.RX_Cnt] = '\0';
        //PrintString1(RX3_Buffer);
        for ( i = 0; i < COM3.RX_Cnt; i++)
        {
          temp[i] = RX3_Buffer[i];
        }
        //PrintString1(temp);
        if(temp[0]==0x11 && temp[1] == 0x22 ){
          if(temp[2] == 0x01){
            runFlagFrom8266 = 1;
          }
          if(temp[2] == 0x02){
            runFlagFrom8266 = 0;
            PWMA_Duty.PWM1_Duty = 0;
            PWMA_Duty.PWM2_Duty = 0;
            xSemaphoreGive(pwmUpdateSignal);
          }
          //PrintString1(temp);
          // itoa(realAngle, temp, 10);
          // PrintString1(temp);
        }
			}
			COM3.RX_Cnt  = 0 ;
      for ( i = 0; i < 15; i++)
        {
          temp[i] = 0;
        }
      
		}

    vTaskDelay(10);
  }
  
}

char* itoa(int num,char* str,int radix)
{
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//索引表
	char temp;//临时变量，交换两个值时用到
    unsigned unum;//存放要转换的整数的绝对值,转换的整数可能是负数
    int i=0,j,k;//i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
 
    //获取要转换的整数的绝对值
    if(radix==10&&num<0)//要转换成十进制数并且是负数
    {
        unum=(unsigned)-num;//将num的绝对值赋给unum
        str[i++]='-';//在字符串最前面设置为'-'号，并且索引加1
    }
    else unum=(unsigned)num;//若是num为正，直接赋值给unum
 
    //转换部分，注意转换后是逆序的
    do
    {
        str[i++]=index[unum%(unsigned)radix];//取unum的最后一位，并设置为str对应位，指示索引加1
        unum/=radix;//unum去掉最后一位
 
    }while(unum);//直至unum为0退出循环
 
    str[i]='\0';//在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
    //将顺序调整过来
    if(str[0]=='-') k=1;//如果是负数，符号不用调整，从符号后面开始调整
    else k=0;//不是负数，全部都要调整
 

    for(j=k;j<=(i-1)/2;j++)//头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
    {
        temp=str[j];//头部赋值给临时变量
        str[j]=str[i-1+k-j];//尾部赋值给头部
        str[i-1+k-j]=temp;//将临时变量的值(其实就是之前的头部值)赋给尾部
    }
 
    return str;//返回转换后的字符串
 
}
// 定义将浮点数转换为字符串的函数
char* floatToString(float num, int precision, char* str) {
    // 在函数开头定义所有新变量
    int offset = 0;
    int intPart;
    float decPart;
    char intStr[20]; // 假设整数部分长度不会超过20位
    int i, j;
    int digit;

    // 处理负数情况
    if (num < 0) {
        str[offset++] = '-';
        num = -num;
    }

    // 提取整数部分
    intPart = (int)num;
    decPart = num - intPart;

    // 将整数部分转换为字符串
    i = 0;
    if (intPart == 0) {
        intStr[i++] = '0';
    } else {
        while (intPart > 0) {
            intStr[i++] = (intPart % 10) + '0';
            intPart /= 10;
        }
        // 反转整数部分字符串
        for (j = 0; j < i / 2; j++) {
            char temp = intStr[j];
            intStr[j] = intStr[i - 1 - j];
            intStr[i - 1 - j] = temp;
        }
    }
    intStr[i] = '\0';

    // 将整数部分拷贝到目标字符串
    for (j = 0; intStr[j] != '\0'; j++) {
        str[offset++] = intStr[j];
    }

    // 添加小数点
    str[offset++] = '.';

    // 将小数部分转换为字符串
    for (i = 0; i < precision; i++) {
        decPart *= 10;
        digit = (int)decPart;
        str[offset++] = '0' + digit;
        decPart -= digit;
    }

    // 添加字符串结束符
    str[offset] = '\0';

    return str;
}

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

float LeftSpeed =0;
float RightSpeed=0;
extern PWMx_Duty PWMA_Duty;
pid_param_t pid1;
pid_param_t pid2;
QueueHandle_t pwmUpdateSignal = NULL;
//串口2接收到的openmv原始数据
char rawAngleFromOPENMV[8] = {0};

extern char *itoa(int num, char *str, int radix);
void motorTEST(void);
char *floatToString(float num, int precision, char *str);

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

    LeftSpeed = Encoder1count * EncoderPerLength ;
    RightSpeed = Encoder2count * EncoderPerLength;

    floatToString(LeftSpeed, 6, output);
    PrintString1(output);
    floatToString(RightSpeed, 6, output);
    PrintString1(output);

    PWMA_Duty.PWM1_Duty+=PidIncCtrl(&pid1, (35 - LeftSpeed )/EncoderPerLength);
    PWMA_Duty.PWM2_Duty+=PidIncCtrl(&pid2, (35 - RightSpeed)/EncoderPerLength);


    T3R = 1;//启动定时器
    T4R = 1;

    xSemaphoreGive(pwmUpdateSignal);
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

    vTaskDelay(10);
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
				strcpy(rawAngleFromOPENMV,RX2_Buffer);
        if(rawAngleFromOPENMV[0]==0x2c && rawAngleFromOPENMV[1] == 0x12)
			}
			COM2.RX_Cnt  = 0 ;
      
		}
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
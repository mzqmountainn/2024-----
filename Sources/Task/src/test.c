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
//Ŀ���ٶ�
int targetSpeed = 10; 
extern PWMx_Duty PWMA_Duty;
pid_param_t pid1;
pid_param_t pid2;
pid_param_t pidLoc;
/// @brief λ��ʽpid����pwm��ֵ
int pwmDeltaPIDloc = 0;
QueueHandle_t pwmUpdateSignal = NULL;
//����2���յ���openmvԭʼ����
char rawAngleFromOPENMV[8] = {0x00/*0x2c,0x12,0x00,0x01,0x01,0x01,0x01,0x01*/};
//�洢����������
int realAngle = 0;
//ͨ��8266��ȡ�����Ƿ�����ָ��
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

//����С���ٶȲ�����pid����
void outputSpeed(void *pvParameters){
  int Encoder1count = 0;//��
  int Encoder2count = 0;//��
  char output[15];
  pvParameters = pvParameters;
  while (1)
  {
    T3R = 0;//ֹͣ��ʱ������
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
    //�����򴮿�1����
    // floatToString(LeftSpeed, 6, output);
    // PrintString1(output);
    // floatToString(RightSpeed, 6, output);
    // PrintString1(output);

    //ͨ��esp8266����
    // floatToString(LeftSpeed, 6, output);
    // PrintString3(output);
    // PrintString3("  ");
    // floatToString(RightSpeed, 6, output);
    // PrintString3(output);
    // PrintString3("\n");

    //PWMA_Duty.PWM1_Duty+=PidIncCtrl(&pid1, (35 - LeftSpeed )/EncoderPerLength);
    //PWMA_Duty.PWM2_Duty+=PidIncCtrl(&pid2, (35 - RightSpeed)/EncoderPerLength);


    T3R = 1;//������ʱ��
    T4R = 1;
    if(runFlagFrom8266){
      pidControl();}
    //xSemaphoreGive(pwmUpdateSignal);
    vTaskDelay(100);
  }
  
}
//pid����
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

//����2����openmv��Ϣ
void openMVgetAngle(void *pvParameters){
  int i = 0;
  char temp[15];
  pvParameters = pvParameters;
  while (1)
  {
    if(COM2.RX_TimeOut > 0 && --COM2.RX_TimeOut == 0 ){
			
			//1.2 �ж��յ������ݳ��� > 0
			if(COM2.RX_Cnt > 0 ){
				
				//1.3 ��ȡ���� :: ����װ�� RX1_Buffer ��������ȥ �õ�֮��ֱ�ӷ���PC��
        // for(i = 0 ; i < COM2.RX_Cnt  ; i++){
				// 	TX1_write2buff(RX2_Buffer[i]);
				// }
				RX2_Buffer[COM2.RX_Cnt] = '\0';
        //PrintString1(RX2_Buffer);
				//strcpy(rawAngleFromOPENMV,RX2_Buffer);  !!!����
        //PrintString1(RX2_Buffer);
        for ( i = 0; i < COM2.RX_Cnt; i++)
        {
          temp[i] = RX2_Buffer[i];
        }
        //PrintString1(temp);
        if(temp[0]==0x2c /*&& temp[1] == 0x12 &&temp[5] == 0x5b&&temp[6] == 0xdd*/){
          realAngle = temp[3];
          if(temp[2] == 2){ //��ͷ����2 ����1
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

//����3����esp8266��Ϣ
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
			
			//1.2 �ж��յ������ݳ��� > 0
			if(COM3.RX_Cnt > 0 ){
				
				//1.3 ��ȡ���� :: ����װ�� RX1_Buffer ��������ȥ �õ�֮��ֱ�ӷ���PC��
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
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//������
	char temp;//��ʱ��������������ֵʱ�õ�
    unsigned unum;//���Ҫת���������ľ���ֵ,ת�������������Ǹ���
    int i=0,j,k;//i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;j����ָʾ����˳��ʱ�Ľ�����
 
    //��ȡҪת���������ľ���ֵ
    if(radix==10&&num<0)//Ҫת����ʮ�����������Ǹ���
    {
        unum=(unsigned)-num;//��num�ľ���ֵ����unum
        str[i++]='-';//���ַ�����ǰ������Ϊ'-'�ţ�����������1
    }
    else unum=(unsigned)num;//����numΪ����ֱ�Ӹ�ֵ��unum
 
    //ת�����֣�ע��ת�����������
    do
    {
        str[i++]=index[unum%(unsigned)radix];//ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
        unum/=radix;//unumȥ�����һλ
 
    }while(unum);//ֱ��unumΪ0�˳�ѭ��
 
    str[i]='\0';//���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
    //��˳���������
    if(str[0]=='-') k=1;//����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
    else k=0;//���Ǹ�����ȫ����Ҫ����
 

    for(j=k;j<=(i-1)/2;j++)//ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
    {
        temp=str[j];//ͷ����ֵ����ʱ����
        str[j]=str[i-1+k-j];//β����ֵ��ͷ��
        str[i-1+k-j]=temp;//����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
    }
 
    return str;//����ת������ַ���
 
}
// ���彫������ת��Ϊ�ַ����ĺ���
char* floatToString(float num, int precision, char* str) {
    // �ں�����ͷ���������±���
    int offset = 0;
    int intPart;
    float decPart;
    char intStr[20]; // �����������ֳ��Ȳ��ᳬ��20λ
    int i, j;
    int digit;

    // ���������
    if (num < 0) {
        str[offset++] = '-';
        num = -num;
    }

    // ��ȡ��������
    intPart = (int)num;
    decPart = num - intPart;

    // ����������ת��Ϊ�ַ���
    i = 0;
    if (intPart == 0) {
        intStr[i++] = '0';
    } else {
        while (intPart > 0) {
            intStr[i++] = (intPart % 10) + '0';
            intPart /= 10;
        }
        // ��ת���������ַ���
        for (j = 0; j < i / 2; j++) {
            char temp = intStr[j];
            intStr[j] = intStr[i - 1 - j];
            intStr[i - 1 - j] = temp;
        }
    }
    intStr[i] = '\0';

    // ���������ֿ�����Ŀ���ַ���
    for (j = 0; intStr[j] != '\0'; j++) {
        str[offset++] = intStr[j];
    }

    // ���С����
    str[offset++] = '.';

    // ��С������ת��Ϊ�ַ���
    for (i = 0; i < precision; i++) {
        decPart *= 10;
        digit = (int)decPart;
        str[offset++] = '0' + digit;
        decPart -= digit;
    }

    // ����ַ���������
    str[offset] = '\0';

    return str;
}

#include "lcd_init.h"
#include	"STC32G_Delay.h"
#include "STC32G_SPI.h"
extern void delay_ms(uint16_t ms);
//不准确延时函数
// void delay_ms(unsigned int ms)
// {                         
// 	unsigned int a;
// 	while(ms)
// 	{
// 		a=1800;
// 		while(a--);
// 		ms--;
// 	}
// }


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{	


	
      SPI_SS_2= 0;     //拉低从机 SS 管脚
      SPI_SetMode(SPI_Mode_Master);	//SPI设置主机模式，开始发送数据
      SPI_WriteByte(dat); //发送串口数据 // 在此处卡死
		SPI_RxCnt = 0;
      SPI_SS_2 = 1;    //拉高从机的 SS 管脚
	


// u8 i;
// LCD_CS_Clr();
// for(i=0;i<8;i++)
// {			  
// 	LCD_SCLK_Clr();
// 	if(dat&0x80)
// 	{
// 	   LCD_MOSI_Set();
// 	}
// 	else
// 	{
// 	   LCD_MOSI_Clr();
// 	}
// 	LCD_SCLK_Set();
// 	dat<<=1;
//	 }	
//   LCD_CS_Set();	
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
}


/******************************************************************************
      函数说明：LCD初始化函数
      入口数据：无
      返回值：  无
******************************************************************************/
void LCD_Init(void)
{
	LCD_RES_Clr();
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);
	LCD_BLK_Set();
	delay_ms(100);
	// //************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	delay_ms(120);              //Delay 120ms 
	// //************* Start Initial Sequence **********// 
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0xc3);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x96);
	LCD_WR_REG(0x36);    // Memory Access Control 
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x48);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0x88);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x28);
	else LCD_WR_DATA8(0xE8);
	LCD_WR_REG(0X3a);
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0Xe6);
	LCD_WR_DATA8(0x0f);
	LCD_WR_DATA8(0xf2);
	LCD_WR_DATA8(0x3f);
	LCD_WR_DATA8(0x4f);
	LCD_WR_DATA8(0x4f);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x0e);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0Xc5);
	LCD_WR_DATA8(0x2a);
	LCD_WR_REG(0Xe0);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x1c);
	LCD_WR_DATA8(0x3b);
	LCD_WR_DATA8(0x55);
	LCD_WR_DATA8(0x4a);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x14);
	LCD_WR_DATA8(0x1c);
	LCD_WR_DATA8(0x1f);
	LCD_WR_REG(0Xe1);
	LCD_WR_DATA8(0xf0);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0a);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x0c);
	LCD_WR_DATA8(0x09);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x49);
	LCD_WR_DATA8(0x0f);
	LCD_WR_DATA8(0x1b);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x1b);
	LCD_WR_DATA8(0x1f);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x3c);
	LCD_WR_REG(0Xf0);
	LCD_WR_DATA8(0x69);
	LCD_WR_REG(0X29);
} 










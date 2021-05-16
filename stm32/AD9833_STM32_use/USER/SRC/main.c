//-----------------------------------------------------------------
// 程序描述: AD9910测试程序
// 作　　者: 凌智电子
// 开始日期: 2020-05-01
// 完成日期: 2020-05-05
// 当前版本: V1.2，增加方波和三角波输出功能
// 历史版本:
//		v1.0:初始版本
//		v1.1:增加并行调制模式
// 调试工具: 凌智STM32核心开发板、2.8寸液晶、LZE_ST_LINK2
// 说　　明: 
//				
//-----------------------------------------------------------------
//												AD9910与STM32F1连接
//						  AD9910                    		STM32F1
//								MRT														PB15
//								CSN														PB14
//								SCK														PB13
//								SDI														PB12
//								IUP														PB11
//								DRH														PB10
//								DRC														PB9

//								PF0														PB8
//								PF1														PA7
//								PF2														PA6
//								OSK														PC7
//
//								GND														JGND
//
//			其中 	DRG功能也可以外接控制信号来控制    控制引脚为 DRC	   DRH
//			 			OSK功能也可以外界控制信号来控制    控制引脚为 OSK
// 如果需要进行并行调制模式, 则还需要对模块的P1接口进行连接
// 不需要并行调制模式, 不需要连接以下P1接口（模式的）

//								F0														PA5
//								F1														PA4
//								D0														PA3
//								D1														PA2
//								D2														PA1
//								D3														PA0
//								D4														PC5
//								D5														PC4
//								D6														PC3
//								D7														PC2
//								D8														PC1
//								D9														PC0
//								D10														PC8
//								D11														PC9
//								D12														PC10
//								D13														PC11
//								D14														PC12
//								D15														PC13
//
//
//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include <stm32f10x.h>
#include "Delay.h"
#include "string.h"
#include "PeripheralInit.h"
#include "AD9910.h"
#include "key.h" 
#include "lze_lcd.h"
#include "stdio.h"
#include "fonts.h"

//unsigned char static DataStr[]=__DATE__;
//unsigned char static TimeStr[]=__TIME__;

u8 key_numb;											// 按键标志位
u8 numb=0;												// 标志位
u8 NB=0;													// AD9910功能标志位
u8 buf[10];												// 显示数组
int i;														// 循环标志位（验证用的）

// 初始化的单频频率
uint32_t fre_data[] = {0, 1, 10, 100, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000, 5000000, 10000000, 50000000, 100000000, 200000000, 300000000, 380000000, 400000000, 420000000};
// 初始化的单频幅度
int amp_data[] = {0,1023,2047,4095,8191,9215,10239,11263,12287,13311,14335,15359,16383};
// 初始化的单频相位
int pha_data[] = {0,90,180,270,360};

// 并口的偏移量
int offset_fre[64] = {0, 1000, 2000, 4000, 8000, 10000, 20000, 40000, 60000};
// 并口的增益
int gain[64] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	
//-----------------------------------------------------------------
// int get_data(int pos_fre, int num[])
//-----------------------------------------------------------------
//
//  函数功能: 将num中的数值合成一个数
//  入口参数: pos_fre数组有效位数，num数据存放的数组
//  返回参数: 返回数据值
//  注意事项: 无
//
//-----------------------------------------------------------------
int get_data(int pos_fre, int num[])
{
	int i = 0, data;
	
	data = num[i];
	i++;
	while(i <= pos_fre)
	{
		data = data*10 + num[i];	
		i++;
	}
	
	return data;	
}
//-----------------------------------------------------------------
// void  clr_data(int pos_fre, int num[])
//-----------------------------------------------------------------
//
//  函数功能: 清除数组里面的数
//  入口参数: pos_fre数组有效位数，num数据存放的数组
//  返回参数: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void  clr_data(int pos_fre, int num[])
{
	int i = 0;
	
	for(; i <= pos_fre; i++)
	{
		num[i] = 0;
	}
}
//9910扫频参数
typedef struct dds_ad9910
{
	int start_parm;
	int stop_parm;
	int rising_step;
	int falling_step;
	int rising_time;
	int falling_time;
}DDS;
u32 get_target(double v, double level) {
	return (u32)((double)v / level* 0x3FFF);
}

#define M 1000000

void USART3_Init(u32 baud) {
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  //声明一个结构体变量，用来初始化GPIO
  //使能串口的RCC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,
                         ENABLE);  //使能UART3所在GPIOB的时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  //串口使用的GPIO口配置
  // Configure USART3 Tx (PB.10) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART3 Rx (PB.11) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //串口中断配置
  // Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // Enable the USART3 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //配置串口
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // Configure USART3
  USART_Init(USART3, &USART_InitStructure);  //配置串口3
  // Enable USART3 Receive interrupts 使能串口接收中断
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  // Enable the USART3
  USART_Cmd(USART3, ENABLE);  //使能串口3
}

void USART3_Puts(char *str) {
  while (*str) {
    USART_SendData(USART3, *str++);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
      ;
  }
}

void set_wave(u32 f, u32 v) {
	AD9910_Singal_Profile_Init();
	AD9910_Singal_Profile_Set(0, f, v, 0);
}

void loop() {
	char buf[64];
	while (1) {
		u32 f = 0;
		for (f = 1 * M; f <= 40 * M; f += M) {
			sprintf(buf, "%lu\r\n", f);
			USART3_Puts(buf);
			set_wave(f, get_target(0.1, 0.49));
		}
	}
}

int main(void)
{
	u32 f = 0;
	u32 target = (u32)((double)0.10 / 0.49 * 0x3FFF); // 10m
	//u32 target = (u32)((double)0.10 / 0.412 * 0x3FFF);  // 20m
	//u32 target = 0x3FFF;
	USART3_Init(115200);
	PeripheralInit();																		// 外设初始化
	AD9910_Init();																			// AD9910时钟的设置和辅助DAC设置 以及AD9910复位
	AD9910_Singal_Profile_Init();																	// 单频PROFILE初始化
	// AD9910_Singal_Profile_Set(0, 10000000, 12, 0);
	// AD9910_Singal_Profile_Set(0, 10000000, 234, 0);
	loop();
	while (1) {
		/*
		for (f = 1000000; f <= 40000000; f+=1000000) {
			AD9910_Singal_Profile_Init();																	// 单频PROFILE初始化
			AD9910_Singal_Profile_Set(0, f, target, 0);									// 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ）
			Delay_50ms(10);
		}*/
		/*
		for (f = 10000000; f <= 40000000; f+=500000) {
			AD9910_Singal_Profile_Init();																	// 单频PROFILE初始化
			AD9910_Singal_Profile_Set(0, f, target, 0);									// 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ）
			Delay_50ms(50);
			//Delay_50ms(1);
		}*/
		/*
		AD9910_Singal_Profile_Init();																	// 单频PROFILE初始化
		//AD9910_Singal_Profile_Set(0, 10000000, 12 + 24 * 2, 0);
		AD9910_Singal_Profile_Set(0, 20000000, target, 0);
		Delay_50ms(50);
		*/
		
		for (f = get_target(0, 0.49); f <= get_target(0.10, 0.49); f+=10) {
			AD9910_Singal_Profile_Init();																	// 单频PROFILE初始化
			AD9910_Singal_Profile_Set(0, 10000000, f, 0);									// 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ）
			Delay_50ms(5);
		}
	}
}
	

//-----------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------

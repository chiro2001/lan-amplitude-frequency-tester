#include <stdio.h>
#include <stdlib.h>
#include <stm32f10x.h>
#include <string.h>
#include "ad9910.h"
#include "delay.h"

//-----------------------------------------------------------------
//												AD9910与STM32F1连接
//						  AD9910 STM32F1 								MRT
//PB15 								CSN
//PB14 								SCK
//PB13 								SDI
//PB12 								IUP
//PB11 								DRH
//PB10 								DRC
//PB9

//								PF0
//PB8 								PF1
//PA7 								PF2
//PA6 								OSK
//PC7
//
//								GND
//JGND
//
//			其中 	DRG功能也可以外接控制信号来控制    控制引脚为 DRC
//DRH 			 			OSK功能也可以外界控制信号来控制    控制引脚为 OSK
// 如果需要进行并行调制模式, 则还需要对模块的P1接口进行连接
// 不需要并行调制模式, 不需要连接以下P1接口（模式的）

//								F0
//PA5 								F1
//PA4
//=====================================================================
//								D0
//PA3 								D1
//PA2 								D2
//PA1 								D3
//PA0 								D4
//PC5 								D5
//PC4 								D6
//PC3 								D7
//PC2 								D8
//PC1 								D9
//PC0 								D10
//PC8 								D11
//PC9 								D12
//PC10 								D13
//PC11 								D14
//PC12 								D15
//PC13

#define M 1000000

void USART1_Init(u32 bound) {
  // GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(
      RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,
      ENABLE);  //使能USART1,GPIOA,C时钟

  // USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  // PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //初始化GPIOA.9

  // USART1_RX      GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             // PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化GPIOA.10

  // Usart1 NVIC 配置
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
                                                   //0-3;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器

  // USART 初始化设置

  USART_InitStructure.USART_BaudRate = bound;  //串口波特率
  USART_InitStructure.USART_WordLength =
      USART_WordLength_8b;  //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;     //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;  //无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式

  USART_Init(USART1, &USART_InitStructure);       //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启串口接受中断
  USART_Cmd(USART1, ENABLE);                      //使能串口1
}
/**
 * USART1发送len个字节.
 * buf:发送区首地址
 * len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
 **/
void USART1_Send_Data(u8 *buf, u16 len) {
  u16 t;
  //GPIO_SetBits(GPIOC, GPIO_Pin_9);
  //  RS485_TX_EN=1;            //设置为发送模式
  for (t = 0; t < len; t++)  //循环发送数据
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
      ;  //循环发送,直到发送完毕
    USART_SendData(USART1, buf[t]);
  }
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    ;
  //GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  //    RS485_TX_EN=0;                //设置为接收模式
}

void USART1_Puts(char *str) {
	USART1_Send_Data((u8 *)str, strlen(str));
}

/*
void USART1_Init(u32 baud) {
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  //声明一个结构体变量，用来初始化GPIO
  //使能串口的RCC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,
                         ENABLE);  //使能UART1所在GPIOA的时钟
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  //串口使用的GPIO口配置
  // Configure USART1 Tx (PA.9) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //串口中断配置
  // Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
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

  // Configure USART1
  USART_Init(USART1, &USART_InitStructure);  //配置串口1
  // Enable USART1 Receive interrupts 使能串口接收中断
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  // Enable the USART1
  USART_Cmd(USART1, ENABLE);  //使能串口1
}

void USART1_Puts(char *str) {
  while (*str) {
    USART_SendData(USART1, *str++);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
      ;
  }
}
*/

u32 get_target(double v, double level) {
  return (u32)((double)v / level * 0x3FFF);
}

double wave_data[] = {
	140, 144, 140, 136, 131, 126, 121, 117, 111, 107, 102, 99, 96, 93, 91, 90, 88, 87, 87, 87, 87,
	89, 91, 93, 95, 98, 102, 107, 112, 116, 121, 125, 127, 127, 125, 120, 115, 106, 99, 92, 85
};

double wave_max_data[] = {
	0, 735, 725, 700, 686, 661, 637, 602, 578, 553, 534, 519, 499, 494, 485, 475, 475, 470, 465, 470, 
	474, 475, 480, 485, 505, 524, 534, 578, 600, 602, 627, 646, 656, 666, 665, 627, 592, 568, 529, 490, 405
};

u32 get_real(double v, u32 f) {
	return (u32)((double)(0.1 * v)/(0.49 * (wave_data[f] / 1000)) * 0x3FFF);
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
      sprintf(buf, "%lu\r\n", (unsigned long)f);
      USART1_Puts(buf);
      // set_wave(f, get_target(0.1, 0.49));
			// set_wave(f, get_real(0.1, f / M));
			// set_wave(f, 0x3fff);
			set_wave(f, get_target(0.107, wave_max_data[f / M] / 1000));
      Delay_50ms(20);
    }
  }
}

void loop_count() {
  char buf[64];
  while (1) {
    u32 f = 0;
    for (f = 1 * M; f <= 40 * M; f += M) {
      sprintf(buf, "%lu\r\n", (unsigned long)f);
      USART1_Puts(buf);
      // set_wave(f, get_target(0.1, 0.49));
			set_wave(f, get_target(0.2, 0.6));
			// set_wave(f, 0x3fff);
			// set_wave(f, get_target(0.107, wave_max_data[f / M] / 1000));
      Delay_50ms(10);
    }
  }
}

int main(void) {
  u32 f = 0;
  u32 target = (u32)((double)0.10 / 0.49 * 0x3FFF);  // 10m
  // u32 target = (u32)((double)0.10 / .412 * 0x3FFF);  // 20m
  // u32 target = 0x3FFF;
  USART1_Init(115200);
	
	GPIO_Init_AD9910();
  AD9910_Init();  // AD9910时钟的设置和辅助DAC设置 以及AD9910复位
  AD9910_Singal_Profile_Init();  // 单频PROFILE初始化
  loop_count();
  while (1) {
    /*
    for (f = 1000000; f <= 40000000; f+=1000000) {
            AD9910_Singal_Profile_Init();
    // 单频PROFILE初始化 AD9910_Singal_Profile_Set(0, f, target, 0);
    // 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ） Delay_50ms(10);
    }*/
    /*
    for (f = 10000000; f <= 40000000; f+=500000) {
            AD9910_Singal_Profile_Init();
    // 单频PROFILE初始化 AD9910_Singal_Profile_Set(0, f, target, 0);
    // 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ） Delay_50ms(50);
            //Delay_50ms(1);
    }*/
    /*
    AD9910_Singal_Profile_Init();
    // 单频PROFILE初始化
    //AD9910_Singal_Profile_Set(0, 10000000, 12 + 24 * 2, 0);
    AD9910_Singal_Profile_Set(0, 20000000, target, 0);
    Delay_50ms(50);
    */
    /*
    for (f = get_target(0, 0.49); f <= get_target(0.10, 0.49); f+=10) {
            AD9910_Singal_Profile_Init();
    // 单频PROFILE初始化 AD9910_Singal_Profile_Set(0, 10000000, f, 0);
    // 设置正弦波信息 （ 通道（0~7） ， 频率 ，幅度 ， 相位 ） Delay_50ms(5);
    }*/
    set_wave(10 * M, get_target(0.1, 0.49));
    Delay_50ms(60);
  }
}

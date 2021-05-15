#include <stm32f10x.h>

/*******************************************************************************
 * 函 数 名         : delay_us
 * 函数功能           : 延时函数，延时us
 * 输    入         : i
 * 输    出         : 无
 *******************************************************************************/
void delay_us(u32 i) {
  u32 temp;
  SysTick->LOAD = 9 * i;  //设置重装数值, 72MHZ时
  SysTick->CTRL = 0X01;  //使能，减到零是无动作，采用外部时钟源
  SysTick->VAL = 0;      //清零计数器
  do {
    temp = SysTick->CTRL;                            //读取当前倒计数值
  } while ((temp & 0x01) && (!(temp & (1 << 16))));  //等待时间到达
  SysTick->CTRL = 0;                                 //关闭计数器
  SysTick->VAL = 0;                                  //清空计数器
}

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
// IO口操作宏定义
#define BITBAND(addr, bitnum) \
  ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long*)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 12)  // 0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12)  // 0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12)  // 0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12)  // 0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12)  // 0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12)  // 0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12)  // 0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8)  // 0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8)  // 0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8)  // 0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8)  // 0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8)  // 0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8)  // 0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8)  // 0x40011E08

// IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)  //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)   //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)  //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)   //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)  //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)   //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)  //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)   //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)  //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)   //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)  //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)   //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)  //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)   //输入

#ifndef _AD9833_H_
#define _AD9833_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
// #include "sys.h"
#define AD9833_FSYNC PBout(12)  //这个是NSS管脚
/******************************************************************************/
/* AD9833                                                                    */
/******************************************************************************/
/* Registers */

#define AD9833_REG_CMD (0 << 14)
#define AD9833_REG_FREQ0 (1 << 14)
#define AD9833_REG_FREQ1 (2 << 14)
#define AD9833_REG_PHASE0 (6 << 13)
#define AD9833_REG_PHASE1 (7 << 13)

/* Command Control Bits */

#define AD9833_B28 (1 << 13)
#define AD9833_HLB (1 << 12)
#define AD9833_FSEL0 (0 << 11)
#define AD9833_FSEL1 (1 << 11)
#define AD9833_PSEL0 (0 << 10)
#define AD9833_PSEL1 (1 << 10)
#define AD9833_PIN_SW (1 << 9)
#define AD9833_RESET (1 << 8)
#define AD9833_SLEEP1 (1 << 7)
#define AD9833_SLEEP12 (1 << 6)
#define AD9833_OPBITEN (1 << 5)
#define AD9833_SIGN_PIB (1 << 4)
#define AD9833_DIV2 (1 << 3)
#define AD9833_MODE (1 << 1)

#define AD9833_OUT_SINUS ((0 << 5) | (0 << 1) | (0 << 3))
#define AD9833_OUT_TRIANGLE ((0 << 5) | (1 << 1) | (0 << 3))
#define AD9833_OUT_MSB ((1 << 5) | (0 << 1) | (1 << 3))
#define AD9833_OUT_MSB2 ((1 << 5) | (0 << 1) | (0 << 3))
/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral and resets the part. */
unsigned char AD9833_Init(void);
/* Sets the Reset bit of the AD9833. */
void AD9833_Reset(void);
/* Clears the Reset bit of the AD9833. */
void AD9833_ClearReset(void);
/* Writes the value to a register. */
void AD9833_SetRegisterValue(unsigned short regValue);
/* Writes to the frequency registers. */
void AD9833_SetFrequency(unsigned short reg, float fout);
/* Writes to the phase registers. */
void AD9833_SetPhase(unsigned short reg, unsigned short val);
/* Selects the Frequency,Phase and Waveform type. */
void AD9833_Setup(unsigned short freq, unsigned short phase,
                  unsigned short type);
void AD9833_SetFrequencyQuick(float fout, unsigned short type);

unsigned char AD9833_SPI_Write(unsigned char* data, unsigned char bytesNumber);
#endif  // _AD9833_H

void SPI2_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  /*******************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /*******这里很关键*************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        // SPI_NSS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

  SPI_InitStructure.SPI_Direction =
      SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;  //设置SPI工作模式:设置为主SPI
  SPI_InitStructure.SPI_DataSize =
      SPI_DataSize_8b;  //设置SPI的数据大小:SPI发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  //选择了串行时钟的稳态:时钟悬空高
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;  //数据捕获于第二个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft      /*Hard  Soft*/
      ;  // NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI_InitStructure.SPI_BaudRatePrescaler =
      SPI_BaudRatePrescaler_128;  //定义波特率预分频的值:波特率预分频值为256
  SPI_InitStructure.SPI_FirstBit =
      SPI_FirstBit_MSB;  //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;  // CRC值计算的多项式
  SPI_Init(
      SPI2,
      &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

  SPI_Cmd(SPI2, ENABLE);  //使能SPI外设

  // SPI2_ReadWriteByte(0xff);//启动传输
}

u8 SPI2_ReadWriteByte(u8 TxData) {
  u8 retry = 0;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) ==
         RESET)  //检查指定的SPI标志位设置与否:发送缓存空标志位
  {
    retry++;
    if (retry > 200) return 0;
  }
  SPI_I2S_SendData(SPI2, TxData);  //通过外设SPIx发送一个数据
  retry = 0;

  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) ==
         RESET)  //检查指定的SPI标志位设置与否:接受缓存非空标志位
  {
    retry++;
    if (retry > 200) return 0;
  }
  return SPI_I2S_ReceiveData(SPI2);  //返回通过SPIx最近接收的数据
}

// #define FCLK 30000000  //设置晶振频率
#define FCLK 25000000  //设置晶振频率
//#define RealFreDat    268435456.0/FCLK//总的公式为
// Fout=（Fclk/2的28次方）*28位寄存器的值
double RealFreDat = 268435456.0 / FCLK;
/********往ad9833写******************************/
unsigned char AD9833_SPI_Write(unsigned char* data, unsigned char bytesNumber) {
  // unsigned char i = 0, j = 0;
  unsigned char i = 0;
  unsigned char writeData[5] = {0, 0, 0, 0, 0};
  AD9833_FSYNC = 0;
  for (i = 0; i < bytesNumber; i++) {
    writeData[i] = data[i + 1];
  }
  for (i = 0; i < bytesNumber; i++) {
    // while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    // //检查指定的SPI标志位设置与否:发送缓存空标志位
    SPI2_ReadWriteByte(writeData[i]);
  }
  AD9833_FSYNC = 1;
  delay_us(5);
  return i;
}

void AD9833_SetRegisterValue(unsigned short regValue) {
  unsigned char data[5] = {0x03, 0x00, 0x00};

  data[1] = (unsigned char)((regValue & 0xFF00) >> 8);
  data[2] = (unsigned char)((regValue & 0x00FF) >> 0);
  AD9833_SPI_Write(data, 2);
}

void AD9833_SetFrequency(unsigned short reg, float fout) {
  unsigned short freqHi = reg;
  unsigned short freqLo = reg;
  unsigned long val = RealFreDat * fout;  // F寄存器的值
  freqHi |= (val & 0xFFFC000) >> 14;
  freqLo |= (val & 0x3FFF);
  AD9833_SetRegisterValue(AD9833_B28);
  AD9833_SetRegisterValue(freqLo);
  AD9833_SetRegisterValue(freqHi);
}

void AD9833_SetFrequencyQuick(float fout, unsigned short type) {
  AD9833_SetFrequency(AD9833_REG_FREQ0, fout * 1000);  // 400 kHz
  AD9833_Setup(AD9833_FSEL0, AD9833_PSEL0, type);
}
unsigned char AD9833_Init(void) {
  SPI2_Init();
  AD9833_SetRegisterValue(AD9833_REG_CMD | AD9833_RESET);
  return (1);
}

void AD9833_Setup(unsigned short freq, unsigned short phase,
                  unsigned short type) {
  unsigned short val = 0;

  val = freq | phase | type;
  AD9833_SetRegisterValue(val);
}

int main() {
  AD9833_Init();
  AD9833_SetFrequencyQuick(10000, AD9833_OUT_SINUS);  //输出频率  HZ    波形类型 /* SINUS TRIANGLE*/
	while (1) {}
  // return 0;
}

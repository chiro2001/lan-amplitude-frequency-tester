#include <stm32f10x.h>

/*******************************************************************************
 * �� �� ��         : delay_us
 * ��������           : ��ʱ��������ʱus
 * ��    ��         : i
 * ��    ��         : ��
 *******************************************************************************/
void delay_us(u32 i) {
  u32 temp;
  SysTick->LOAD = 9 * i;  //������װ��ֵ, 72MHZʱ
  SysTick->CTRL = 0X01;  //ʹ�ܣ����������޶����������ⲿʱ��Դ
  SysTick->VAL = 0;      //���������
  do {
    temp = SysTick->CTRL;                            //��ȡ��ǰ������ֵ
  } while ((temp & 0x01) && (!(temp & (1 << 16))));  //�ȴ�ʱ�䵽��
  SysTick->CTRL = 0;                                 //�رռ�����
  SysTick->VAL = 0;                                  //��ռ�����
}

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
// IO�ڲ����궨��
#define BITBAND(addr, bitnum) \
  ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long*)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO�ڵ�ַӳ��
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

// IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)  //���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)   //����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)  //���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)   //����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)  //���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)   //����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)  //���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)   //����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)  //���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)   //����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)  //���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)   //����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)  //���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)   //����

#ifndef _AD9833_H_
#define _AD9833_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
// #include "sys.h"
#define AD9833_FSYNC PBout(12)  //�����NSS�ܽ�
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /*******����ܹؼ�*************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        // SPI_NSS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
  GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

  SPI_InitStructure.SPI_Direction =
      SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;  //����SPI����ģʽ:����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize =
      SPI_DataSize_8b;  //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  //ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;  //���ݲ����ڵڶ���ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft      /*Hard  Soft*/
      ;  // NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
  SPI_InitStructure.SPI_BaudRatePrescaler =
      SPI_BaudRatePrescaler_128;  //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
  SPI_InitStructure.SPI_FirstBit =
      SPI_FirstBit_MSB;  //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;  // CRCֵ����Ķ���ʽ
  SPI_Init(
      SPI2,
      &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

  SPI_Cmd(SPI2, ENABLE);  //ʹ��SPI����

  // SPI2_ReadWriteByte(0xff);//��������
}

u8 SPI2_ReadWriteByte(u8 TxData) {
  u8 retry = 0;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) ==
         RESET)  //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
  {
    retry++;
    if (retry > 200) return 0;
  }
  SPI_I2S_SendData(SPI2, TxData);  //ͨ������SPIx����һ������
  retry = 0;

  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) ==
         RESET)  //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
  {
    retry++;
    if (retry > 200) return 0;
  }
  return SPI_I2S_ReceiveData(SPI2);  //����ͨ��SPIx������յ�����
}

// #define FCLK 30000000  //���þ���Ƶ��
#define FCLK 25000000  //���þ���Ƶ��
//#define RealFreDat    268435456.0/FCLK//�ܵĹ�ʽΪ
// Fout=��Fclk/2��28�η���*28λ�Ĵ�����ֵ
double RealFreDat = 268435456.0 / FCLK;
/********��ad9833д******************************/
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
    // //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
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
  unsigned long val = RealFreDat * fout;  // F�Ĵ�����ֵ
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
  AD9833_SetFrequencyQuick(10000, AD9833_OUT_SINUS);  //���Ƶ��  HZ    �������� /* SINUS TRIANGLE*/
	while (1) {}
  // return 0;
}

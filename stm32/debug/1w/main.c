#include <stdio.h>
#include <stm32f10x.h>

u32 fac_ms, fac_us;
void delay_init() {
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  //ѡ���ⲿʱ��  HCLK/8
  fac_us = SystemCoreClock / 8000000;                    //Ϊϵͳʱ�ӵ�1/8
  fac_ms = (u16)fac_us * 1000;  //��OS��,����ÿ��ms��Ҫ��systickʱ����
}

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

void delay_ms(u16 nms) {
  u32 temp;
  SysTick->LOAD = (u32)nms * fac_ms;  //ʱ�����(SysTick->LOADΪ24bit)
  SysTick->VAL = 0x00;                //��ռ�����
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //��ʼ����
  do {
    temp = SysTick->CTRL;
  } while ((temp & 0x01) && !(temp & (1 << 16)));  //�ȴ�ʱ�䵽��
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
  SysTick->VAL = 0X00;                             //��ռ�����
}

void delay1() {
  u32 temp;
  SysTick->LOAD = (u32)fac_ms;  //ʱ�����(SysTick->LOADΪ24bit)
  SysTick->VAL = 0x00;          //��ռ�����
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //��ʼ����
  do {
    temp = SysTick->CTRL;
  } while ((temp & 0x01) && !(temp & (1 << 16)));  //�ȴ�ʱ�䵽��
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
  SysTick->VAL = 0X00;                             //��ռ�����
}

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
// IO�ڲ����궨��
#define BITBAND(addr, bitnum) \
  ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
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

u32 ADC_Read() {
  u32 res = 1;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  delay_us(10);
  res = (u32)ADC_GetConversionValue(ADC1);
  res = (res * 3300) >> 12;
  return res;
}

void USART3_Init(u32 baud) {
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  //����һ���ṹ�������������ʼ��GPIO
  //ʹ�ܴ��ڵ�RCCʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,
                         ENABLE);  //ʹ��UART3����GPIOB��ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  //����ʹ�õ�GPIO������
  // Configure USART3 Tx (PB.10) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure USART3 Rx (PB.11) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //�����ж�����
  // Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // Enable the USART3 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //���ô���
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  // Configure USART3
  USART_Init(USART3, &USART_InitStructure);  //���ô���3
  // Enable USART3 Receive interrupts ʹ�ܴ��ڽ����ж�
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  // Enable the USART3
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
}

void Init() {
  delay_init();
  GPIO_InitTypeDef GPIO_InitStructure, GPIO_InitStructure_;
  ADC_InitTypeDef ADC_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_0);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure_.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure_.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure_);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);

  ADC_DiscModeChannelCountConfig(ADC1, 1);
  ADC_DiscModeCmd(ADC1, ENABLE);

  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1))
    ;

  USART3_Init(115200);
}

void USART3_Puts(char *str) {
  while (*str) {
    USART_SendData(USART3, *str++);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
      ;
  }
}

int main() {
  u32 v;
  char buf[128];
  Init();
  while (1) {
    v = ADC_Read();
    sprintf(buf, "%.3lf\r\n", ((double)v / 1000));
    USART3_Puts(buf);
		delay_ms(1000);
  }
  // return 0;
}

//-----------------------------------------------------------------
// ��������: AD9910���Գ���
// ��������: ���ǵ���
// ��ʼ����: 2020-05-01
// �������: 2020-05-05
// ��ǰ�汾: V1.2�����ӷ��������ǲ��������
// ��ʷ�汾:
//		v1.0:��ʼ�汾
//		v1.1:���Ӳ��е���ģʽ
// ���Թ���: ����STM32���Ŀ����塢2.8��Һ����LZE_ST_LINK2
// ˵������: 
//				
//-----------------------------------------------------------------
//												AD9910��STM32F1����
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
//			���� 	DRG����Ҳ������ӿ����ź�������    ��������Ϊ DRC	   DRH
//			 			OSK����Ҳ�����������ź�������    ��������Ϊ OSK
// �����Ҫ���в��е���ģʽ, ����Ҫ��ģ���P1�ӿڽ�������
// ����Ҫ���е���ģʽ, ����Ҫ��������P1�ӿڣ�ģʽ�ģ�

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
// ͷ�ļ�����
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

u8 key_numb;											// ������־λ
u8 numb=0;												// ��־λ
u8 NB=0;													// AD9910���ܱ�־λ
u8 buf[10];												// ��ʾ����
int i;														// ѭ����־λ����֤�õģ�

// ��ʼ���ĵ�ƵƵ��
uint32_t fre_data[] = {0, 1, 10, 100, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000, 5000000, 10000000, 50000000, 100000000, 200000000, 300000000, 380000000, 400000000, 420000000};
// ��ʼ���ĵ�Ƶ����
int amp_data[] = {0,1023,2047,4095,8191,9215,10239,11263,12287,13311,14335,15359,16383};
// ��ʼ���ĵ�Ƶ��λ
int pha_data[] = {0,90,180,270,360};

// ���ڵ�ƫ����
int offset_fre[64] = {0, 1000, 2000, 4000, 8000, 10000, 20000, 40000, 60000};
// ���ڵ�����
int gain[64] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	
//-----------------------------------------------------------------
// int get_data(int pos_fre, int num[])
//-----------------------------------------------------------------
//
//  ��������: ��num�е���ֵ�ϳ�һ����
//  ��ڲ���: pos_fre������Чλ����num���ݴ�ŵ�����
//  ���ز���: ��������ֵ
//  ע������: ��
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
//  ��������: ��������������
//  ��ڲ���: pos_fre������Чλ����num���ݴ�ŵ�����
//  ���ز���: ��
//  ע������: ��
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
//9910ɨƵ����
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
	PeripheralInit();																		// �����ʼ��
	AD9910_Init();																			// AD9910ʱ�ӵ����ú͸���DAC���� �Լ�AD9910��λ
	AD9910_Singal_Profile_Init();																	// ��ƵPROFILE��ʼ��
	// AD9910_Singal_Profile_Set(0, 10000000, 12, 0);
	// AD9910_Singal_Profile_Set(0, 10000000, 234, 0);
	loop();
	while (1) {
		/*
		for (f = 1000000; f <= 40000000; f+=1000000) {
			AD9910_Singal_Profile_Init();																	// ��ƵPROFILE��ʼ��
			AD9910_Singal_Profile_Set(0, f, target, 0);									// �������Ҳ���Ϣ �� ͨ����0~7�� �� Ƶ�� ������ �� ��λ ��
			Delay_50ms(10);
		}*/
		/*
		for (f = 10000000; f <= 40000000; f+=500000) {
			AD9910_Singal_Profile_Init();																	// ��ƵPROFILE��ʼ��
			AD9910_Singal_Profile_Set(0, f, target, 0);									// �������Ҳ���Ϣ �� ͨ����0~7�� �� Ƶ�� ������ �� ��λ ��
			Delay_50ms(50);
			//Delay_50ms(1);
		}*/
		/*
		AD9910_Singal_Profile_Init();																	// ��ƵPROFILE��ʼ��
		//AD9910_Singal_Profile_Set(0, 10000000, 12 + 24 * 2, 0);
		AD9910_Singal_Profile_Set(0, 20000000, target, 0);
		Delay_50ms(50);
		*/
		
		for (f = get_target(0, 0.49); f <= get_target(0.10, 0.49); f+=10) {
			AD9910_Singal_Profile_Init();																	// ��ƵPROFILE��ʼ��
			AD9910_Singal_Profile_Set(0, 10000000, f, 0);									// �������Ҳ���Ϣ �� ͨ����0~7�� �� Ƶ�� ������ �� ��λ ��
			Delay_50ms(5);
		}
	}
}
	

//-----------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------

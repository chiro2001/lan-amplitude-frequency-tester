
#include  <stm32f10x.h>
#include "systemconfig.h"
#include "common.h"

/*******************************************************************************
* ��������       : RCC_Configuration
* ��������       : ��������ʱ��.
* ��ڲ���       : ��
* ���ڲ���       : ��
*******************************************************************************/
void RCC_Configuration(void)
{
 

	//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); 

	//ʹ�ܸ��ù���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//��ʱ��ʼ��
	Delay_Init(); 
					 
}

/*******************************************************************************
* ��������       : GPIO_Configuration
* ��������       : ������Ŀ�������õ���GPIO
* ��ڲ���       : ��
* ���ڲ���       : ��
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//ʹ��SWJ����ֹ JTAG

 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  //LED1,LED2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		  //key
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	
	//����UART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
                                                
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;              //TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	 

}

/*******************************************************************************
* ��������       : NVIC_Configuration
* ��������       : ����������.
* ��ڲ���       : ��
* ���ڲ���       : ��
*******************************************************************************/
void NVIC_Configuration(void)
{
	//EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	/*����USART1�ж����� */
#ifdef EN_USART_RX	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

	/*����USART3�ж����� */	

//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 	//���ô���3�ж�
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);


}


void SystemConfiguration(void)
{	
	RCC_Configuration(); 

	GPIO_Configuration(); 	

	NVIC_Configuration();
}




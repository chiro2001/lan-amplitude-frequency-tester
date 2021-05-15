#include "stm32f10x.h"
#include "systemconfig.h" 
#include "common.h"
#include "ad9833.h"

#define LED1	BIT_ADDR(GPIOB_ODR_Addr,0)	  
#define LED2	BIT_ADDR(GPIOB_ODR_Addr,1)
#define KEY		BIT_ADDR(GPIOB_IDR_Addr,2)

#define CLK 			BIT_ADDR(GPIOB_ODR_Addr,14)	  
int main(void)
{	

	 u8 i;

   SystemConfiguration();		    //ϵͳ��ʼ��

	LED1=0; //��
	LED2=1;	//��

	AD9833_Init_GPIO(); //AD9833 GPIO ��ʼ��

//	AD9833_WaveSeting(8000000.5,0,TRI_WAVE,0 );//1.0005KHz,Ƶ�ʼĴ���0�����ǲ���� ,����λ0
/*	AD9833_WaveSeting(5000,0,SQU_WAVE,90);	//5KHz,		Ƶ�ʼĴ���0��������� 	,����λ90 */
/*	AD9833_WaveSeting(100000,0,SIN_WAVE,0 );//100KHz,	Ƶ�ʼĴ���0�����Ҳ���� ,����λ0 */
	

   	AD9833_WaveSeting(1000.1,0,TRI_WAVE,0);


	AD9833_AmpSet(200); //���÷�ֵ����ֵ��� 255


	while(1)
	{
		LED1 = !LED1_READ; //LED1 ȡ�� 
		Delay_ms(300);	   //��ʱ 300ms
	}
}





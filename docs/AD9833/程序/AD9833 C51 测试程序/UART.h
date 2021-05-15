//////////////////////////////////
//


#include "STC15F2K60S2.H"
#include <string.h>
#include <intrins.h>



/** Initlize IE TCON SCON function **/

void initCOM(void)
{
	TMOD=0x20;    //��ʱ��T1ʹ�ù�����ʽ2
    TH1=250;     //���ó�ֵ
    TH0=250;
    TR1=1;      //��ʼ��ʱ
    PCON=0x80;     //SMOD=1��
    SCON=0x50;     //������ʽ1��������9600bit/s,�������
    TI=1;
}

/** Send a char to MC **/

void sendChar(unsigned char ch)
{
	SBUF = ch;
	while(TI == 0);
	TI = 0;
}

//����һ���ַ���
void sendStr(unsigned char *s)
{
    while(*s!='\0')         
    {
        sendChar(*s);
        s++;
    }
}

/** COM receive message interrupt function**/

void inteReceive (void) interrupt 4 using 1
{
	if(RI == 1)
	{
		RI = 0;
	}
}
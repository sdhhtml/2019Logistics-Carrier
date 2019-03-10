#ifndef __TIM1_H_
#define __TIM1_H_
#include <stm32f10x.h>


void TIM1_Init(void);
void TIM1Control(void);
void TIM1_test(void);

void get_left(void);//ץȡ���
void put_down(void);//��߷���
void reset(void);//��ߵĸ�λ

void dj_oled(void);

void get_mid(void);//ץȡ�м�
void put_down2(void);//�м����
void reset_s(void);//�м�ĸ�λ

void order_getmid(void);//���ڻ�ȡ�м���ɫ
void order_getright(void);//���ڻ�ȡ�ұ���ɫ
void order_get(void);//���ڻ�ȡ��ɫ˳��
void get_code(void);//���ڶ�ά����Ϣ

void tim_getmid(void);//�����ǿ��м�
void tim_getright(void);//�����ſ��ұ�
void tim_reset1(void);//�������ɫ��λ

void tim_getcode(void);//�����ſ���ά��
void tim_reset2(void);//�������ά�븴λ

void Arbitrary_steering_gear(int wh ,int ch);
#define dj_delay 30
#endif
/*
void put_down(void)
{
for(tim1ch4=405;tim1ch4<450;tim1ch4+=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(100);
	}
	tim1ch4=450;
	TIM_SetCompare4(TIM1,tim1ch4);
for(tim1ch1=355;tim1ch1<660;tim1ch1+=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(100);
	}
	tim1ch1=660;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=605;tim8ch1>545;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(100);
}
	tim8ch1=545;
TIM_SetCompare1(TIM8,tim8ch1);	
for(tim1ch4=450;tim1ch4>360;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(100);
	}
	tim1ch4=360;
	TIM_SetCompare4(TIM1,tim1ch4);
for(tim8ch2=380;tim8ch2<475;tim8ch2+=20)
{	TIM_SetCompare2(TIM8,tim8ch2);
	Delay_ms(100);
}
	Delay_ms(100);
	tim8ch2=475;
TIM_SetCompare2(TIM8,tim8ch2);		
}
*/

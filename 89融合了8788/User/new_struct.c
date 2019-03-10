#include "TIM1.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "oled.h"
#include "steer.h"
#include "new_struct.h"
void Grasp_left(void)//(ץȡ��߶�ά�뷽��
{
	Arbitrary_steering_gear(620 ,1);
	Delay_ms(200);
	//Delay_ms(1500);
	Arbitrary_steering_gear(520,3);
	//Delay_ms(500);
	Arbitrary_steering_gear(740,2);
	//Delay_ms(500);
	Arbitrary_steering_gear(510,4);
	Delay_ms(500);
	//TIM_SetCompare2(TIM8,370);
	//Delay_ms(500);
	Arbitrary_steering_gear(840,2);
	//Arbitrary_steering_gear(745,1);	
}
void Grasp_mid(void)//(ץȡ�м䣩
{
	Arbitrary_steering_gear(760,1);
	//Delay_ms(1500);
	Arbitrary_steering_gear(470,3);
	//Delay_ms(500);
	Arbitrary_steering_gear(800,2);
	//Delay_ms(100);
	Arbitrary_steering_gear(510,4);
	Delay_ms(500);
	//TIM_SetCompare2(TIM8,370);
	//Delay_ms(500);
	Arbitrary_steering_gear(840,2);
	//Arbitrary_steering_gear(745,1);	
}
void Grasp_right(void)//(ץȡ�ұ߷���������
{
	Arbitrary_steering_gear(870,1);
	//Delay_ms(1500);
	Arbitrary_steering_gear(500,3);
	//Delay_ms(500);
	Arbitrary_steering_gear(760,2);
	//Delay_ms(100);
	Arbitrary_steering_gear(510,4);
	Delay_ms(500);
	//TIM_SetCompare2(TIM8,370);
//	Delay_ms(500);
	Arbitrary_steering_gear(840,2);
	//Arbitrary_steering_gear(745,1);		
}
void Place_left(void)//������߶�ά�뷽��
{
	Arbitrary_steering_gear(1590,1);
	//Delay_ms(500);
	Arbitrary_steering_gear(630,3);
	Arbitrary_steering_gear(590,2);
//	Delay_ms(1000);
	Delay_ms(500);
	Arbitrary_steering_gear(655,4);	
}
void Place_mid(void)//�����м�
{
	Arbitrary_steering_gear(1430,1);
	//Delay_ms(500);
	Arbitrary_steering_gear(570,3);
	Arbitrary_steering_gear(620,2);
//	Delay_ms(1000);
	//Arbitrary_steering_gear(405,2);
	Delay_ms(500);
	Arbitrary_steering_gear(655,4);	
}
void Place_right(void)//�����ұ߷���������
{
	Arbitrary_steering_gear(1230,1);
	//Delay_ms(500);
	Arbitrary_steering_gear(610,3);
//	Delay_ms(200);
	Arbitrary_steering_gear(610,2);
	//Delay_ms(1000);
	Delay_ms(500);
	Arbitrary_steering_gear(655,4);		
}
void DJ_reset(void)
{
	Arbitrary_steering_gear(350,3);
	Arbitrary_steering_gear(1140,2);
	Arbitrary_steering_gear(1070,1);
}
void Observing_color(void)//�۲�״̬
{
	Arbitrary_steering_gear(760,1);
	Arbitrary_steering_gear(450,3);
	Arbitrary_steering_gear(840,2);	
}
void sehuan_fa(void)//�۲췢����ɫ��
{
	Arbitrary_steering_gear(1270,1);
	Arbitrary_steering_gear(450,3);
	Arbitrary_steering_gear(830,2);	
}
void sehuan_zhong(void)//�۲��м�ɫ��
{
	Arbitrary_steering_gear(1460,1);
	Arbitrary_steering_gear(450,3);
	Arbitrary_steering_gear(830,2);		
}
void Observing_qrcode(void)//�ı�Ƕȿ���ά��
{
	Arbitrary_steering_gear(760,1);
	Arbitrary_steering_gear(490,3);
	Arbitrary_steering_gear(830,2);		
}

#include "led.h"
struct GLOBAL_FLAGS status = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct HONGWAI_FLAGS hongwaiflags = {0,0,0,0,0,0,0,0,0,0,0,0};
void led_init(void)
{
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/

	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(GPIOE, &GPIO_InitStructure);	


		/* �ر�����led��	*/

		GPIO_SetBits(GPIOE, GPIO_Pin_8);
		GPIO_SetBits(GPIOE, GPIO_Pin_9);
		GPIO_SetBits(GPIOE, GPIO_Pin_10);
}


/**/


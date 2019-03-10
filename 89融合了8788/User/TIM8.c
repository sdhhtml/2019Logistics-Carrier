#include "TIM8.h"
#include "TIM1.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
extern u16 tim8ch1;
extern u16 tim8ch2;
static void ADVANCE_TIM_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // ����Ƚ�ͨ�� GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
static void ADVANCE_TIM_Mode_Config(void)
{
  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);

/*--------------------ʱ���ṹ���ʼ��-------------------------*/
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=999;	
	// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= 239;	//144mhz
	//1000 000
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	TIM_UpdateRequestConfig(TIM8,TIM_UpdateSource_Regular); //����ֻ�м��������Ϊ�����ж�
	/*--------------------����ȽϽṹ���ʼ��-------------------*/		
	
	// ����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// ���ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// �������ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable; 
	// ����ռ�ձȴ�С
	TIM_OCInitStructure.TIM_Pulse = 350;
	// ���ͨ����ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// ���ͨ�����е�ƽ��������
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	// �������ͨ�����е�ƽ��������
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 655;
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 475;
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	
	
	// ʹ�ܼ�����
	TIM_Cmd(TIM8, ENABLE);	
	// �����ʹ�ܣ���ʹ�õ���ͨ�ö�ʱ��ʱ����䲻��Ҫ
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

}
void TIM8_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}
void TIM8_Init(void)
{
ADVANCE_TIM_GPIO_Config();	
	ADVANCE_TIM_Mode_Config();
	//TIM8_NVIC_Config();
}
/*
3�Ŷ�� TIM8CH1-C6  990�� 240��
4�Ŷ�� TIM8CH2-C7  700���� 500��
*/
void TIM8Control(void)
{
	if(status.TIM8CH1_up==1)//q
	{
		tim8ch1+=10;if(tim8ch1>=990)tim8ch1=990;
		TIM_SetCompare1(TIM8,tim8ch1);	
		printf("tim8ch1=%d\r\n",tim8ch1);
		status.TIM8CH1_up=0;
	}
	if(status.TIM8CH1_down==1)//w
	{
		tim8ch1-=10;if(tim8ch1<=160)tim8ch1=160;
		TIM_SetCompare1(TIM8,tim8ch1);		
		printf("tim8ch1=%d\r\n",tim8ch1);
		status.TIM8CH1_down=0;
	}	
	
	
	if(status.TIM8CH2_up==1)//e
	{
		tim8ch2+=10;if(tim8ch2>=990)tim8ch2=990;
		TIM_SetCompare2(TIM8,tim8ch2);		
		printf("tim8ch2=%d\r\n",tim8ch2);
		status.TIM8CH2_up=0;
	}
	if(status.TIM8CH2_down==1)//r
	{
		tim8ch2-=10;if(tim8ch2<=300)tim8ch2=300;
		TIM_SetCompare2(TIM8,tim8ch2);		
		printf("tim8ch2=%d\r\n",tim8ch2);		
		status.TIM8CH2_down=0;
	}
}
void TIM8_test(void)
{
	int i=0;
	for(i=280;i<420;i+=20)
	{
		TIM_SetCompare1(TIM8,i);
		
		printf("%d\r\n",i);
	//	TIM_SetCompare2(TIM8,i);
//		TIM_SetCompare3(TIM8,i);
		Delay_ms(440);
		
	}	
	for(;i>280;i-=20)
	{
		TIM_SetCompare1(TIM8,i);
	//	TIM_SetCompare2(TIM8,i);
//		TIM_SetCompare3(TIM8,i);
		printf("%d\r\n",i);
		Delay_ms(200);
				
	}
	
}
/**/

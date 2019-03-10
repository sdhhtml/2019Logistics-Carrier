#include "TIM3.h"

unsigned int CaptureNumberA1=0;
unsigned int CaptureNumberA2=0;
unsigned int CaptureNumberB1=0;
unsigned int CaptureNumberB2=0;

static void ENCODER_TIM3_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ʹ�ܶ�ʱ��ͨ������GPIOʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  /* ���ö�ʱ��ͨ��1����ģʽ�� */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}
static void TIM3_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* ѡ���ж����ȼ�������Ϊ2����ռʽ���ȼ���2�������ȼ������Բο�misc.h�ļ��˽�������� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����TIM3_IRQ�ж�Ϊ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	/* ������ռʽ���ȼ�Ϊ0 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* ���������ȼ�Ϊ2 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	/* ʹ���ⲿ�ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**
  * ��������: ����TIMx�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
/*
 * TIMxCLK/CK_PSC --> TIMxCNT --> TIMx_ARR --> TIMxCNT ���¼���
 *                    TIMx_CCR(��ƽ�����仯)
 * �ź�����=(TIMx_ARR +1 ) * ʱ������
 * ռ�ձ�=TIMx_CCR/(TIMx_ARR +1)
 *    _______    ______     _____      ____       ___        __         _
 * |_|       |__|      |___|     |____|    |_____|   |______|  |_______| |________|
 *
 */
static void ENCODER_TIM3_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;

  /* ʹ�ܶ�ʱ��ʱ�� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  /* ��ʱ����������ʼ�� */		 
  /* ��ʱ����: ENCODER_TIM_Period+1 */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
  /* ����Ԥ��Ƶ��ENCODER_TIM_Prescaler,�������Ƶ�ʣ�72MHz/(ENCODER_TIM_Prescaler+1)/(ENCODER_TIM_Period+1) */
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  /* ����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���) */
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
  /* ���ϼ���ģʽ */
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* ��ʼ��TIM5���벶����� */
  /* CC1S=01 	ѡ������� IC1ӳ�䵽TI1�� */
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
  /* �����ز��� */
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;	
  /* ӳ�䵽TI1�� */
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  /* ���������Ƶ,����Ƶ  */
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           
  /* IC1F=0000 ���������˲��� ���˲� */
	TIM_ICInitStructure.TIM_ICFilter    = 0x00;                         
	TIM_ICInit(TIM3, &TIM_ICInitStructure);  
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_3;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);	
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_4;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);		
  /* ����NVIC */
  TIM3_NVIC_Configuration();
  
  /* ��������ж� ,����CC1IE�����ж�	 */
  TIM_ITConfig(TIM3,TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_CC4, ENABLE);
	
  /* ʹ�ܶ�ʱ�� */
  TIM_Cmd(TIM3, ENABLE);							
}
/**
  * ��������: TIMxͨ��1���벶���ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void TIM3_Init(void)
{
	ENCODER_TIM3_GPIO_Config();
	ENCODER_TIM3_Configuration();	
}
void TIM3_IRQHandler(void)
{
 	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET )              //�����ж�
	{
     /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
    CaptureNumberA1++;  // ÿ��һ�������ؼ�һ
	}
 	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET )              //�����ж�
	{
     /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);
    CaptureNumberA2++;  // ÿ��һ�������ؼ�һ
	}
 	if(TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET )              //�����ж�
	{
     /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3,TIM_IT_CC3);
    CaptureNumberB1++;  // ÿ��һ�������ؼ�һ
	}
 	if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET )              //�����ж�
	{
     /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM3,TIM_IT_CC4);
    CaptureNumberB2++;  // ÿ��һ�������ؼ�һ
	}	
}
/**/

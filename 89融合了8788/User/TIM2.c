#include "TIM2.h"
#include "bsp_usart.h"
uint32_t TIM_PscCLK = 1000000;
//TIM2
// ��ʱ�����벶���û��Զ�������ṹ�嶨��

TIM_ICUserValueTypeDef ultrasonic3 = {0,0,0,0,0,0};
TIM_ICUserValueTypeDef ultrasonic4 = {0,0,0,0,0,0};
// �ж����ȼ�����
static void TIM2_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // �����ж���Ϊ0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn ;	
		// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void TIM2_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // ���벶��ͨ�� GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
 	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;				 //LED0-->PB.5 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	 GPIO_Init(GPIOE, &GPIO_InitStructure);					 
	 GPIO_ResetBits(GPIOB,GPIO_Pin_14);						 
	 GPIO_ResetBits(GPIOB,GPIO_Pin_15);						 					 		 
}


///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *	TIM_CounterMode			     TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
// ARR ���Զ���װ�ؼĴ�����ֵ
// CLK_cnt����������ʱ�ӣ����� Fck_int / (psc+1) = 72M/(psc+1)
// PWM �źŵ����� T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// ռ�ձ�P=CCR/(ARR+1)

static void TIM2_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

/*--------------------ʱ���ṹ���ʼ��-------------------------*/	
  
	// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=0XFFFF;	
	// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= 71;	
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/*--------------------���벶��ṹ���ʼ��-------------------*/	
	
	// �������벶���ͨ������Ҫ���ݾ����GPIO������
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	// ���벶���źŵļ�������
	TIM_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;
	// ����ͨ���Ͳ���ͨ����ӳ���ϵ����ֱ���ͷ�ֱ������
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	// �������Ҫ��������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	// �������Ҫ��������źŵ��˲�ϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;
	// ��ʱ�����벶���ʼ��
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_IT_CC3);	
  // �������ºͲ����ж�  
	TIM_ITConfig (TIM2, TIM_IT_Update | TIM_IT_CC3, ENABLE );
	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_IT_CC4);	
  // �������ºͲ����ж�  
	TIM_ITConfig (TIM2, TIM_IT_Update | TIM_IT_CC4, ENABLE );
	
	// ʹ�ܼ�����
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_Init(void)
{
	TIM2_GPIO_Config();
	TIM2_NVIC_Config();
	TIM2_Mode_Config();		
}
void TIM2_IRQHandler(void)
{
	// ��Ҫ��������źŵ����ڴ��ڶ�ʱ�������ʱʱ����ʱ���ͻ���������������ж�
	// ���ʱ��������Ҫ�������Ķ�ʱ���ڼӵ������źŵ�ʱ������ȥ
	if ( TIM_GetITStatus ( TIM2, TIM_IT_Update) != RESET )               
	{	
		ultrasonic3.Capture_Period ++;
		ultrasonic4.Capture_Period ++;		
		TIM_ClearITPendingBit ( TIM2, TIM_FLAG_Update ); 		
	}


	// �����ز����ж�3
	if ( TIM_GetITStatus (TIM2, TIM_IT_CC3 ) != RESET)
	{
		// ��һ�β���
		if ( ultrasonic3.Capture_StartFlag == 0 )
		{
			// ��������0
			TIM_SetCounter ( TIM2, 0 );
			// �Զ���װ�ؼĴ������±�־��0
			ultrasonic3.Capture_Period = 0;
      // �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
			ultrasonic3.Capture_CcrValue = 0;

			// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);
      // ��ʼ�����׼��1			
			ultrasonic3.Capture_StartFlag = 1;			
		}
		// �½��ز����ж�
		else // �ڶ��β���
		{
			// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
			ultrasonic3.Capture_CcrValue = 
			TIM_GetCapture3(TIM2);

			// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising);
      // ��ʼ�����־��0		
			ultrasonic3.Capture_StartFlag = 0;
      // ������ɱ�־��1			
			ultrasonic3.Capture_FinishFlag = 1;		
		}

		TIM_ClearITPendingBit (TIM2,TIM_IT_CC3);	    
	}	
	// �����ز����ж�4
	if ( TIM_GetITStatus (TIM2, TIM_IT_CC4 ) != RESET)
	{
		// ��һ�β���
		if ( ultrasonic4.Capture_StartFlag == 0 )
		{
			// ��������0
			TIM_SetCounter ( TIM2, 0 );
			// �Զ���װ�ؼĴ������±�־��0
			ultrasonic4.Capture_Period = 0;
      // �沶��ȽϼĴ�����ֵ�ı�����ֵ��0			
			ultrasonic4.Capture_CcrValue = 0;

			// ����һ�β���������֮�󣬾ͰѲ����������Ϊ�½���
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);
      // ��ʼ�����׼��1			
			ultrasonic4.Capture_StartFlag = 1;			
		}
		// �½��ز����ж�
		else // �ڶ��β���
		{
			// ��ȡ����ȽϼĴ�����ֵ�����ֵ���ǲ��񵽵ĸߵ�ƽ��ʱ���ֵ
			ultrasonic4.Capture_CcrValue = 
			TIM_GetCapture4(TIM2);

			// ���ڶ��β����½���֮�󣬾ͰѲ����������Ϊ�����أ��ÿ����µ�һ�ֲ���
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising);
      // ��ʼ�����־��0		
			ultrasonic4.Capture_StartFlag = 0;
      // ������ɱ�־��1			
			ultrasonic4.Capture_FinishFlag = 1;		
		}

		TIM_ClearITPendingBit (TIM2,TIM_IT_CC4);	    
	}		
}
void CSB3(void)
{
						if(ultrasonic3.Capture_FinishFlag == 1)
							{
								// ����ߵ�ƽʱ��ļ�������ֵ
								ultrasonic3.time = ultrasonic3.Capture_Period * (0XFFFF+1) + 
											 (ultrasonic3.Capture_CcrValue+1);
								ultrasonic3.distance=ultrasonic3.time%TIM_PscCLK*340/2/1000;
								// ��ӡ�ߵ�ƽ����ʱ��
								//printf ( "\r\n��øߵ�ƽ����ʱ�䣺%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK );
								//Lcd_printf4(ultrasonic3.distance,0);
								printf ( "\r\n��������3��%d mm\r\n",ultrasonic3.distance );
								ultrasonic3.Capture_FinishFlag = 0;			
							}	
}
void CSB4(void)
{
						if(ultrasonic4.Capture_FinishFlag == 1)
							{
								// ����ߵ�ƽʱ��ļ�������ֵ
								ultrasonic4.time = ultrasonic4.Capture_Period * (0XFFFF+1) + 
											 (ultrasonic4.Capture_CcrValue+1);
								ultrasonic4.distance=ultrasonic4.time%TIM_PscCLK*340/2/1000;
								// ��ӡ�ߵ�ƽ����ʱ��
								//printf ( "\r\n��øߵ�ƽ����ʱ�䣺%d.%d s\r\n",time/TIM_PscCLK,time%TIM_PscCLK );
								//Lcd_printf4(ultrasonic4.distance,30);
								printf ( "\r\n��������4��%d mm\r\n",ultrasonic4.distance );
								ultrasonic4.Capture_FinishFlag = 0;			
							}	
}
/**/

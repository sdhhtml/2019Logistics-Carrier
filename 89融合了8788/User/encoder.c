#include "encoder.h"
#include "stm32f10x.h"
#include "key.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "mpu9250.h"
#include "mpu6050.h"
extern unsigned int CaptureNumberA1;//��
extern unsigned int CaptureNumberA2;//��
extern unsigned int CaptureNumberB1;
extern unsigned int CaptureNumberB2;
static void NVIC_Configuration1(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  /* ����NVICΪ���ȼ���1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  /* ������ռ���ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* ���������ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* ʹ���ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  /* �����ж�Դ������2������ʹ������������� */  
  NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn;
  NVIC_Init(&NVIC_InitStructure);


}
void Encoder_init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	EXTI_InitTypeDef EXTI_initStructure;
	NVIC_Configuration1();
	//ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	//�����õ����ж���·������Ӧ�Ĵ���ģʽ�ʹ���ʱ��
	EXTI_initStructure.EXTI_Line = EXTI_Line6;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger =EXTI_Trigger_Rising;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	EXTI_initStructure.EXTI_Line = EXTI_Line7;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger =EXTI_Trigger_Rising;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	//����GPIO�ܽŵ��ж���·

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);	
	//����жϱ�־����ֹһ���ж�ͨ���ͽ����ж�

	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
	
	//����NVIC��EXTI���ж�ͨ��

	NVIC_EnableIRQ(EXTI9_5_IRQn);

}
//void EXTI9_5_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line6) == SET){
//		if((GPIOA->IDR & GPIO_Pin_6) == SET){				//ע�⣺���ﲻ����==SET���жϣ���ΪSET = !RESET,Ҳ����˵SET�����ִ���ֵ�Ƿ���
////			if((GPIOA->IDR & GPIO_Pin_6) == SET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
//////				while((GPIOA->IDR & GPIO_Pin_6) == SET);		//�ȴ���ť�ͷ�

////			}
//			CaptureNumberA1+=1;
//		}
//	}
//		EXTI_ClearITPendingBit(EXTI_Line7);	
//	
//	if(EXTI_GetITStatus(EXTI_Line7) == SET){
//		if((GPIOA->IDR & GPIO_Pin_7) == SET){				//ע�⣺���ﲻ����==SET���жϣ���ΪSET = !RESET,Ҳ����˵SET�����ִ���ֵ�Ƿ���
////			if((GPIOA->IDR & GPIO_Pin_6) == SET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
//////				while((GPIOA->IDR & GPIO_Pin_6) == SET);		//�ȴ���ť�ͷ�

////			}
//			CaptureNumberA2+=1;
//		}
//	}
//		EXTI_ClearITPendingBit(EXTI_Line7);	
//}
//TIM3	CH1-A6 CH2-A7

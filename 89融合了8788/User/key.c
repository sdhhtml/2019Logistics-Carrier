#include "key.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "mpu9250.h"
#include "mpu6050.h"
extern u16 tim1ch1;
extern u16 tim1ch4;
extern double angleoffset;
extern double uart5_angle;
//KEY	D3 D4 E7 D12 A12 F12 F13 F15
//D3 D4 D12 
//A12
//E7
//F12 F13 F15
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  /* ����NVICΪ���ȼ���1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* ������ռ���ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* ���������ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  /* ʹ���ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  /* �����ж�Դ������2������ʹ������������� */  
  NVIC_InitStructure.NVIC_IRQChannel =EXTI3_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannel =EXTI4_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;
  NVIC_Init(&NVIC_InitStructure);	
}
void key_init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	EXTI_InitTypeDef EXTI_initStructure;
	NVIC_Configuration();
	//ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_AFIO,ENABLE);	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);

	GPIO_initStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_12;	
	GPIO_Init(GPIOD,&GPIO_initStructure);
	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOE,&GPIO_initStructure);
	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_Init(GPIOF,&GPIO_initStructure);
	//�����õ����ж���·������Ӧ�Ĵ���ģʽ�ʹ���ʱ��
	EXTI_initStructure.EXTI_Line = EXTI_Line3;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger =EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line4;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line7;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line12;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);

	EXTI_initStructure.EXTI_Line = EXTI_Line13;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	
	EXTI_initStructure.EXTI_Line = EXTI_Line15;
	EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_initStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_initStructure);
	//����GPIO�ܽŵ��ж���·
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource4);		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF,GPIO_PinSource15);
	
	//����жϱ�־����ֹһ���ж�ͨ���ͽ����ж�
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line7);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line15);
	//����NVIC��EXTI���ж�ͨ��
	NVIC_EnableIRQ(EXTI3_IRQn);	
	NVIC_EnableIRQ(EXTI4_IRQn);	
	NVIC_EnableIRQ(EXTI9_5_IRQn);	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//�����жϴ�����
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) == SET){
		if((GPIOD->IDR & GPIO_Pin_3) == RESET){				//ע�⣺���ﲻ����==SET���жϣ���ΪSET = !RESET,Ҳ����˵SET�����ִ���ֵ�Ƿ���
			Delay_ms(10);										//�ӳ�10ms
			if((GPIOD->IDR & GPIO_Pin_3) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOD->IDR & GPIO_Pin_3) == RESET);		//�ȴ���ť�ͷ�
				digitalToggle(GPIOE,GPIO_Pin_8);
				status.start=1;
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line7);	
}
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) == SET){
		if((GPIOD->IDR & GPIO_Pin_4) == RESET){				//ע�⣺���ﲻ����==SET���жϣ���ΪSET = !RESET,Ҳ����˵SET�����ִ���ֵ�Ƿ���
			Delay_ms(10);										//�ӳ�10ms
			if((GPIOD->IDR & GPIO_Pin_4) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOD->IDR & GPIO_Pin_4) == RESET);		//�ȴ���ť�ͷ�
				digitalToggle(GPIOE,GPIO_Pin_8);
				status.start=1;
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line7);	
}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) == SET){
		if((GPIOE->IDR & GPIO_Pin_7) == RESET){				//ע�⣺���ﲻ����==SET���жϣ���ΪSET = !RESET,Ҳ����˵SET�����ִ���ֵ�Ƿ���
			Delay_ms(10);										//�ӳ�10ms
			if((GPIOE->IDR & GPIO_Pin_7) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOE->IDR & GPIO_Pin_7) == RESET);		//�ȴ���ť�ͷ�
				digitalToggle(GPIOE,GPIO_Pin_8);
			}
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line7);	
}



void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12) == SET){
		if((GPIOA->IDR & GPIO_Pin_12) == RESET){
			Delay_ms(10);										//�ӳ�20ms
			if((GPIOA->IDR & GPIO_Pin_12) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOA->IDR & GPIO_Pin_12) == RESET);
				digitalToggle(GPIOE,GPIO_Pin_8);
		}
		if((GPIOD->IDR & GPIO_Pin_12) == RESET){
			Delay_ms(10);										//�ӳ�20ms
			if((GPIOD->IDR & GPIO_Pin_12) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOD->IDR & GPIO_Pin_12) == RESET);
				digitalToggle(GPIOE,GPIO_Pin_9);
		}			
		if((GPIOF->IDR & GPIO_Pin_12) == RESET){
			Delay_ms(10);										//�ӳ�20ms
			if((GPIOF->IDR & GPIO_Pin_12) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOF->IDR & GPIO_Pin_12) == RESET);
				digitalToggle(GPIOE,GPIO_Pin_10);
		}
	}
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
		else if(EXTI_GetITStatus(EXTI_Line13) == SET){
		if((GPIOF->IDR & GPIO_Pin_13) == RESET){
			Delay_ms(10);										//�ӳ�20ms
			if((GPIOF->IDR & GPIO_Pin_13) == RESET){			//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOF->IDR & GPIO_Pin_13) == RESET);
				digitalToggle(GPIOE,GPIO_Pin_8);
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	else if(EXTI_GetITStatus(EXTI_Line15) == SET){
		if((GPIOF->IDR & GPIO_Pin_15)== RESET){
			Delay_ms(10);										//�ӳ�20ms
			if((GPIOF->IDR & GPIO_Pin_15)== RESET){				//�ټ��һ�°����ĵ�ƽ״̬������ʱ���Ǵ��ڵ͵�ƽ״̬��˵�������������ڶ���������
				while((GPIOF->IDR & GPIO_Pin_15)== RESET);
				digitalToggle(GPIOE,GPIO_Pin_8);
				
			}
		}
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}
}
	}
/**/

#include "uart4.h"
#include "TIM4.h"
#include "led.h"
#include "TIM1.h"
extern unsigned char motor_mode;
//���ڽ�������
unsigned char UART_RxBuffer4[UART_RX_BUFFER_SIZE4];
//���ڽ�������ָ��
unsigned char UART_RxPtr4;
//extern unsigned char P_DATA;
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration4(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void UART_Config4(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	// �򿪴��������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = 115200;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(UART4, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration4();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(UART4, ENABLE);	    
}
void UART4_IRQHandler(void)
{
  unsigned char ucTemp;
	if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(UART4);
		if(ucTemp==0x30){motor_mode=0;}//0
		if(ucTemp==0x31){motor_mode=1;}//1
		if(ucTemp==0x32){motor_mode=2;}//2
		if(ucTemp==0x33){motor_mode=3;}//3
	  if(ucTemp==0x34){motor_mode=4;}//4
		if(ucTemp==0x35){status.TIM1CH1_up = 1;
		//P_DATA+=1;if(P_DATA>=30)P_DATA=30;IncPIDInit();//PID��ʼ��
		}//5
		if(ucTemp==0x36){status.TIM1CH1_down=1;
		//P_DATA-=1;if(P_DATA<=0)P_DATA=0;IncPIDInit();//PID��ʼ��
		}//6
		if(ucTemp==0x37){status.TIM1CH4_up=1;}//7
		if(ucTemp==0x38){status.TIM1CH4_down=1;}//8
		if(ucTemp==0x71){status.TIM8CH1_up=1;}//q
		if(ucTemp==0x77){status.TIM8CH1_down=1;}//w
		if(ucTemp==0x65){status.TIM8CH2_up=1;}//e
		if(ucTemp==0x72){status.TIM8CH2_down=1;}//r
//		if(ucTemp==0x39){status.TIM8CH1_up=1;}//9
//		LED2_TOGGLE;									
   // USART_SendData(UART4,ucTemp);    
	}	 
}

/**/

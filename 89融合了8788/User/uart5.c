#include "uart5.h"
#include "bsp_usart.h"
#include "TIM4.h"
#include "led.h"
#include "oled.h"
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
	
unsigned char uart5_receice[8];
unsigned char uart5_mode=0;
unsigned char uart5_count=0;
unsigned short int qd_code=0;//��ά��
unsigned char order_1=0;//���˳��
unsigned char order_2=0;//ɫ��˳��
extern double uart5_angle;
unsigned char color_mid=0;
unsigned char color_right=0;
unsigned short int color_mode=0;
extern unsigned char flag_mid;
extern unsigned char flag_right;
extern unsigned char flag_code;
unsigned char color_shunjian=0;//˲����ɫ
static void NVIC_Configuration5(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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
void UART_Config5(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	// �򿪴��������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = 9600;
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
	USART_Init(UART5, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration5();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(UART5, ENABLE);	   
}	
//63 6F 31
//73 74 33 32 31 
//28 27 33 32 31 27 2C 29
void UART5_IRQHandler(void)
{
  unsigned char ucTemp;
//	int i=0;
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(UART5);

								if(ucTemp==0x28)uart5_mode=1;
								else if(ucTemp==0x27&&uart5_mode==1)uart5_mode=2;//��ʼ���ܶ�ά��
								if(uart5_mode==2)
								{
									uart5_receice[uart5_count]=ucTemp;
									uart5_count++;
								}
								if(uart5_count==3&&uart5_mode==2)//��ά��˳��
									{
											uart5_mode=0;
											uart5_count=0;
										if(uart5_receice[1]==0x31&&uart5_receice[2]==0x32)
											qd_code=123;//123
										else if(uart5_receice[1]==0x31&&uart5_receice[2]==0x33)
											qd_code=132;//132
										else if(uart5_receice[1]==0x32&&uart5_receice[2]==0x31)
											qd_code=213;//213
										else if(uart5_receice[1]==0x32&&uart5_receice[2]==0x33)
											qd_code=231;//231
										else if(uart5_receice[1]==0x33&&uart5_receice[2]==0x31)
											qd_code=312;//312
										else if(uart5_receice[1]==0x33&&uart5_receice[2]==0x32)
											qd_code=321;//321	
										flag_code=1;
									}
								if(ucTemp==0x63)uart5_mode=3;
								else if(ucTemp==0x6f&&uart5_mode==3)uart5_mode=4;
								if(uart5_mode==4)
								{
									uart5_receice[uart5_count]=ucTemp;
									uart5_count++;
								}									
								if(uart5_count==2&&uart5_mode==4)//˲����ɫ
									{
											uart5_mode=0;
											uart5_count=0;
										if(uart5_receice[1]==0x31)
											color_shunjian=1;
										else if(uart5_receice[1]==0x32)
											color_shunjian=2;
										else if(uart5_receice[1]==0x33)
											color_shunjian=3;
									}								

								if(ucTemp==0x73)uart5_mode=5;
								else if(ucTemp==0x74&&uart5_mode==5)uart5_mode=6;
								if(uart5_mode==6)
								{
									uart5_receice[uart5_count]=ucTemp;
									uart5_count++;
								}									
								if(uart5_count==3&&uart5_mode==6)//�������˳��
									{
											uart5_mode=0;
											uart5_count=0;
										if(uart5_receice[1]==0x31&&uart5_receice[2]==0x32)
											order_1=1;//123
										else if(uart5_receice[1]==0x31&&uart5_receice[2]==0x33)
											order_1=2;//132
										else if(uart5_receice[1]==0x32&&uart5_receice[2]==0x31)
											order_1=3;//213
										else if(uart5_receice[1]==0x32&&uart5_receice[2]==0x33)
											order_1=4;//231
										else if(uart5_receice[1]==0x33&&uart5_receice[2]==0x31)
											order_1=5;//312
										else if(uart5_receice[1]==0x33&&uart5_receice[2]==0x32)
											order_1=6;//321		
									}									
	}	 
}
void Color_judgement(void)
{
	if(color_mid==1&&color_right==2)//����������ά��
	{
		color_mode=213;
	}
	else if(color_mid==1&&color_right==3)//����������ά��
	{
		color_mode=312;
	}
	else if(color_mid==2&&color_right==1)//����������ά��
	{
		color_mode=123;
	}
	else if(color_mid==2&&color_right==3)//����������ά��
	{
		color_mode=321;
	}
	else if(color_mid==3&&color_right==1)//����������ά��
	{
		color_mode=132;
	}
	else if(color_mid==3&&color_right==2)//����������ά��
	{
		color_mode=231;
	}	
	else if(color_mid==0||color_right==0)//����������ά��
	{
		if(color_mid==0&&color_right==0)
		{
			color_mode=123;
		}
		else if(color_mid==0&&color_right==1)
		{
			color_mode=123;
		}
		else if(color_mid==0&&color_right==2)
		{
			color_mode=213;
		}
		else if(color_mid==0&&color_right==3)
		{
			color_mode=312;
		}		
		else if(color_mid==1&&color_right==0)
		{
			color_mode=213;
		}
		else if(color_mid==2&&color_right==0)
		{
			color_mode=123;
		}
		else if(color_mid==3&&color_right==0)
		{
			color_mode=132;
		}			
	}	
	
	OLED_ShowNum(byte(2),line5,color_mode,4,6); 
}
/*
void UART5_IRQHandler(void)
{
  unsigned char ucTemp;
	int i=0;
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(UART5);
//		Usart_SendByte( USART1, ucTemp);
//		LED1_TOGGLE;
						if(status.uart5==0)
						{	
								if(ucTemp==0x0D)uart5_mode=1;
								if(uart5_mode==1)
								{
									uart5_receice[uart5_count]=ucTemp;
									uart5_count++;
								}
									if(uart5_count==2&&uart5_receice[1]!=0x0a)
									{
											uart5_mode=0;
											uart5_count=0;
									}
								
								if(uart5_count==8)
								{
										uart5_mode=0;
										uart5_count=0;
										uart5_angle=(uart5_receice[2]-0x30)*100+(uart5_receice[3]-0x30)*10+(uart5_receice[4]-0x30)+0.1*(uart5_receice[6]-0x30);
											
												
										for(i=2;i<5;i++)
											{
												uart5_receice[i]=0;
											}
										LED2_TOGGLE;	
										status.uart5=1;
								}
						}   // USART_SendData(UART4,ucTemp);    
	}	 
}
*/
/**/

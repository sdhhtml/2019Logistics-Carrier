
#include "bsp_usart3.h"
#include "bsp_usart.h"
#include "led.h"
#include "oled.h"
//���ڽ�������
unsigned char UART_RxBuffer3[UART_RX_BUFFER_SIZE3];
//���ڽ�������ָ��
unsigned char UART_RxPtr3;
unsigned char rxtx_mode;
int xx=0,yy=0,ii=0,jj=0;
extern unsigned short int JG_distance;
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration3(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
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
void USART_Config3(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// �򿪴��������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = 38400;
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
	USART_Init(USART3, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration3();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(USART3, ENABLE);	    
}
#if 0
void USART3_IRQHandler(void)

{	
    unsigned char data;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{
		data = USART_ReceiveData(USART3);
		
		if(data==0x5b){rxtx_mode=1;}//[
		else if(data==0x2c){rxtx_mode=2;}//,[100,100]
		else if(data==0x5d){rxtx_mode=3;}//]
		if(rxtx_mode==1){
			UART_RxBuffer3[ii]=(data-0x30);
			ii++;
		}
		else if(rxtx_mode==2){
			UART_RxBuffer3[jj]=(data-0x30);
			jj++;
		}
		else if(rxtx_mode==3){
			if(ii==4){xx=UART_RxBuffer3[1]*100+UART_RxBuffer3[2]*10+UART_RxBuffer3[3];}
				else if(ii==3){xx=UART_RxBuffer3[1]*10+UART_RxBuffer3[2];}
					else if(ii==2){xx=UART_RxBuffer3[1];}
			if(jj==8){yy=UART_RxBuffer3[5]*100+UART_RxBuffer3[6]*10+UART_RxBuffer3[7];}
				else if(jj==7){yy=UART_RxBuffer3[5]*10+UART_RxBuffer3[6];}
					else if(jj==6){yy=UART_RxBuffer3[5];}
			ii=0;
			jj=4;
					status.flag3 = TRUE;
		}
    }	 

}
#endif

void USART3_IRQHandler(void)
{	
	static u8 step=0;
	static u8 buf[8];
	u16 SPEED_X = 0;
	u16 SPEED_Y = 0;
	u16 HIGH = 10;
	u8 Res;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{
	Res = USART_ReceiveData(USART3);	
	switch(step)
		{
		case 0:
			if(Res == 0xaa)
			{
				buf[0] = Res;
				step = 1;
			}
			else
			{
				step = 0;	
			}
				break;
			case 1:
			if(Res == 0x55)
			{
				buf[1] = Res;
				step = 2;
			}
			else
			{
				step = 0;	
			}
				break;			
			case 2:
				buf[2] = Res;
				step = 3;
				break;
		  case 3:
				buf[3] = Res;
				step = 4;
				break;
			case 4:
				buf[4] = Res;
				step = 5;
				break;
			case 5:
				buf[5] = Res;
				step = 6;
				break;
			case 6:
				buf[6] = Res;
				step = 7;
				break;
			case 7:
				buf[7] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6];	
				if(buf[7]==Res)
				{
					SPEED_X = (s8)buf[2];
					SPEED_Y = (s8)buf[3];
					HIGH = (buf[5] + (buf[6]<<8));
					JG_distance=HIGH;
					//printf("Distance be measured is %d \n",HIGH);

				}
				step = 0;
				break;
			default:
				step = 0;
				break;
	}
	}
	
}


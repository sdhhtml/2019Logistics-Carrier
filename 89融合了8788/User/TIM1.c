#include "TIM1.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "oled.h"
extern u16 tim1ch1;
extern u16 tim1ch4;
extern u16 tim8ch1;
extern u16 tim8ch2;
extern unsigned char order_1;
extern unsigned char color_mid;
extern unsigned char color_right;
extern unsigned char qd_code;
extern float yaww;
static void ADVANCE_TIM_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // ����Ƚ�ͨ�� GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
static void ADVANCE_TIM_Mode_Config(void)
{
  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

/*--------------------ʱ���ṹ���ʼ��-------------------------*/
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=9999;	//300hz
	// ����CNT��������ʱ�� = Fck_int/(psc+1)

	TIM_TimeBaseStructure.TIM_Prescaler=143;//	2 000 000
//	TIM_TimeBaseStructure.TIM_Prescaler=719;//1 000 000	
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/*--------------------����ȽϽṹ���ʼ��-------------------*/		

	// ����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// ���ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// �������ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable; 
	// ����ռ�ձȴ�С
	TIM_OCInitStructure.TIM_Pulse = 1070;
	// ���ͨ����ƽ��������
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// ���ͨ�����е�ƽ��������
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	// �������ͨ�����е�ƽ��������
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OCInitStructure.TIM_Pulse = 1140;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);


	

	// �����ʹ�ܣ���ʹ�õ���ͨ�ö�ʱ��ʱ����䲻��Ҫ
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	// ʹ�ܼ�����
	TIM_Cmd(TIM1, ENABLE);		
}
void TIM1_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���					 
}
void TIM1_Init(void)
{
	ADVANCE_TIM_GPIO_Config();
	ADVANCE_TIM_Mode_Config();		
	//TIM1_NVIC_Config();
}
/*
1�Ŷ�� TIM1CH1-A8  ��240 530 ץ840   545
2�Ŷ�� TIM1CH4-A11 ��240 530 ̧840   280ƽ
*/
void TIM1Control(void)
{
	if(status.TIM1CH1_up==1)//5
	{
		tim1ch1+=10;if(tim1ch1>=1700)tim1ch1=1700;
		TIM_SetCompare1(TIM1,tim1ch1);
		printf("tim1ch1=%d\r\n",tim1ch1);
		status.TIM1CH1_up=0;
	}
	if(status.TIM1CH1_down==1)//6
	{
		tim1ch1-=10;if(tim1ch1<=620)tim1ch1=620;
		TIM_SetCompare1(TIM1,tim1ch1);		
		printf("tim1ch1=%d\r\n",tim1ch1);
		status.TIM1CH1_down=0;
	}	
	if(status.TIM1CH4_up==1)//7
	{
		tim1ch4+=10;if(tim1ch4>=1260)tim1ch4=1260;
		TIM_SetCompare4(TIM1,tim1ch4);	
		printf("tim1ch4=%d\r\n",tim1ch4);		
		status.TIM1CH4_up=0;
	}
	if(status.TIM1CH4_down==1)//8
	{
		tim1ch4-=10;if(tim1ch4<=140)tim1ch4=140;
		TIM_SetCompare4(TIM1,tim1ch4);		
		printf("tim1ch4=%d\r\n",tim1ch4);
		status.TIM1CH4_down=0;
	}	
}
void TIM1_test(void)//
{
	int i=0;
	for(tim1ch1=280;tim1ch1<500;tim1ch1+=10)
	{
	TIM_SetCompare2(TIM8,tim1ch1);
		dj_oled();
		printf("%d\r\n",i);
		Delay_ms(500);
		
	}
	for(;tim1ch1>280;tim1ch1-=20)
	{
		TIM_SetCompare2(TIM8,tim1ch1);
		dj_oled();
		printf("%d\r\n",i);
		Delay_ms(500);
				
	}
	
	
}


void get_left(void)
{
for(tim1ch1=480;tim1ch1>355;tim1ch1-=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=355;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=810;tim8ch1>605;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=605;
TIM_SetCompare1(TIM8,tim8ch1);

for(tim1ch4=700;tim1ch4>405;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=405;
	TIM_SetCompare4(TIM1,tim1ch4);
	
for(tim8ch2=475;tim8ch2>380;tim8ch2-=20)
			{	TIM_SetCompare2(TIM8,tim8ch2);
				Delay_ms(100);
			}
				Delay_ms(100);
			tim8ch2=380;
			TIM_SetCompare2(TIM8,tim8ch2);	

	TIM_SetCompare2(TIM8,tim8ch2);
for(tim1ch4=405;tim1ch4<450;tim1ch4+=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(100);
	}
	tim1ch4=450;
	TIM_SetCompare4(TIM1,tim1ch4);			
for(tim1ch1=355;tim1ch1<660;tim1ch1+=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(100);
	}
	tim1ch1=660;
	TIM_SetCompare1(TIM1,tim1ch1);			
}
void put_down(void)
{
for(tim8ch1=605;tim8ch1>545;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(100);
}
	tim8ch1=545;
TIM_SetCompare1(TIM8,tim8ch1);	
for(tim1ch4=450;tim1ch4>360;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(100);
	}
	tim1ch4=360;
	TIM_SetCompare4(TIM1,tim1ch4);
for(tim8ch2=380;tim8ch2<475;tim8ch2+=20)
{	TIM_SetCompare2(TIM8,tim8ch2);
	Delay_ms(100);
}
	Delay_ms(100);
	tim8ch2=475;
TIM_SetCompare2(TIM8,tim8ch2);		


}
void reset(void)
{
	for(tim1ch4=360;tim1ch4<450;tim1ch4+=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
		tim1ch4=450;
		TIM_SetCompare4(TIM1,tim1ch4);
		
	for(tim8ch1=545;tim8ch1<810;tim8ch1+=20)
	{	TIM_SetCompare1(TIM8,tim8ch1);
		Delay_ms(100);
	}
		tim8ch1=810;
	TIM_SetCompare1(TIM8,tim8ch1);	
	
	for(tim1ch1=660;tim1ch1>480;tim1ch1-=20)	
		{
		TIM_SetCompare1(TIM1,tim1ch1);
		Delay_ms(100);
		}
		tim1ch1=480;
		TIM_SetCompare1(TIM1,tim1ch1);	
	for(tim1ch4=450;tim1ch4<700;tim1ch4+=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
		tim1ch4=700;
		TIM_SetCompare4(TIM1,tim1ch4);		
}
/////////////////////////////////////////////////////
void get_mid(void)
{
for(tim1ch1=480;tim1ch1>305;tim1ch1-=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=305;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=810;tim8ch1>645;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=645;
TIM_SetCompare1(TIM8,tim8ch1);

for(tim1ch4=700;tim1ch4>415;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=415;
	TIM_SetCompare4(TIM1,tim1ch4);
	
for(tim8ch2=475;tim8ch2>380;tim8ch2-=20)
			{	TIM_SetCompare2(TIM8,tim8ch2);
				Delay_ms(100);
			}
				Delay_ms(100);
			tim8ch2=380;
			TIM_SetCompare2(TIM8,tim8ch2);	

	TIM_SetCompare2(TIM8,tim8ch2);	
}
//
void put_down2(void)
{
for(tim1ch4=415;tim1ch4<500;tim1ch4+=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(100);
	}
	tim1ch4=480;
	TIM_SetCompare4(TIM1,tim1ch4);
for(tim1ch1=305;tim1ch1<660;tim1ch1+=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(100);
	}
	tim1ch1=660;
	TIM_SetCompare1(TIM1,tim1ch1);

for(tim8ch1=645;tim8ch1>630;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(100);
}
	tim8ch1=630;
TIM_SetCompare1(TIM8,tim8ch1);	

	for(tim1ch4=480;tim1ch4>390;tim1ch4-=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
	tim1ch4=390;
	TIM_SetCompare4(TIM1,tim1ch4);
		for(tim8ch2=380;tim8ch2<475;tim8ch2+=20)
		{	TIM_SetCompare2(TIM8,tim8ch2);
			Delay_ms(100);
		}
			Delay_ms(100);
			tim8ch2=475;
			TIM_SetCompare2(TIM8,tim8ch2);	
}

void reset_s(void)
{
	for(tim1ch4=390;tim1ch4<450;tim1ch4+=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
		tim1ch4=450;
		TIM_SetCompare4(TIM1,tim1ch4);
		
	for(tim8ch1=630;tim8ch1<810;tim8ch1+=20)
	{	TIM_SetCompare1(TIM8,tim8ch1);
		Delay_ms(100);
	}
		tim8ch1=810;
	TIM_SetCompare1(TIM8,tim8ch1);	
	
	for(tim1ch1=660;tim1ch1>480;tim1ch1-=20)	
		{
		TIM_SetCompare1(TIM1,tim1ch1);
		Delay_ms(100);
		}
		tim1ch1=480;
		TIM_SetCompare1(TIM1,tim1ch1);	
	for(tim1ch4=450;tim1ch4<700;tim1ch4+=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
		tim1ch4=700;
		TIM_SetCompare4(TIM1,tim1ch4);		
}
void order_getmid(void)
{
	
//	Usart_SendByte( UART5, 0X31);
	while(status.uart5!=1){Usart_SendByte( UART5, 0X31);Delay_ms(100);}
	status.uart5=0;
}
void order_getright(void)
{
	while(status.uart5!=1){Usart_SendByte( UART5, 0X31);Delay_ms(100);}
	status.uart5=0;
}
void order_get(void)
{
	
										if(color_mid==2&&color_right==3)
											order_1=1;//123
										else if(color_mid==3&&color_right==2)
											order_1=2;//132
										else if(color_mid==1&&color_right==3)
											order_1=3;//213
										else if(color_mid==3&&color_right==1)
											order_1=4;//231
										else if(color_mid==1&&color_right==2)
											order_1=5;//312
										else if(color_mid==2&&color_right==1)
											order_1=6;//321		
}
void dj_oled(void)
{
				OLED_ShowNum(byte(0),line6,tim1ch1,6,6); //Y
				OLED_ShowNum(byte(6),line6,tim1ch4,6,6); //X								
				OLED_ShowNum(byte(0),line7,tim8ch1,6,6); //X
				OLED_ShowNum(byte(6),line7,tim8ch2,6,6); //Y			
						if(yaww<0)
		{
			OLED_ShowString(byte(8),line8,"YAW:-",6);
			yaww = -1*yaww;
		}else
		{
			OLED_ShowString(byte(8),line8,"YAW: ",6);
		}
			OLED_ShowNum(byte(12),line8,yaww*10,4,6); //�����	
}
void get_code(void)
{
	while(status.uart5!=1){Usart_SendByte( UART5, 0X35);Delay_ms(100);}
	status.uart5=0;
//	OLED_ShowNum(byte(12),line2,uart5_angle*10,4,6);
}
void tim_getmid(void)//�����ǿ��м�
/*
305
445��495
690
475
*/
{
for(tim1ch1=480;tim1ch1>305;tim1ch1-=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=305;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=810;tim8ch1>690;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=690;
TIM_SetCompare1(TIM8,tim8ch1);

for(tim1ch4=700;tim1ch4>445;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=445;
	TIM_SetCompare4(TIM1,tim1ch4);
	
for(tim1ch4=445;tim1ch4<495;tim1ch4+=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=495;
	TIM_SetCompare4(TIM1,tim1ch4);		
}
void tim_getright(void)//�����ſ��ұ�
/*
305
445��495
690
475
										355
										430
										665
										475
*/
{
for(tim1ch1=305;tim1ch1<355;tim1ch1+=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=355;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=690;tim8ch1>665;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=665;
TIM_SetCompare1(TIM8,tim8ch1);

for(tim1ch4=495;tim1ch4>430;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=430;
	TIM_SetCompare4(TIM1,tim1ch4);	
}
void tim_getcode(void)//�����ſ���ά��
	/*
315
635
780
475
*/
{
	for(tim1ch1=480;tim1ch1>315;tim1ch1-=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=315;
	TIM_SetCompare1(TIM1,tim1ch1);
for(tim8ch1=810;tim8ch1>780;tim8ch1-=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=780;
TIM_SetCompare1(TIM8,tim8ch1);	
for(tim1ch4=700;tim1ch4>635;tim1ch4-=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=635;
	TIM_SetCompare4(TIM1,tim1ch4);
}
void tim_reset1(void)//�������ɫ��λ
/*
										355
										430
										665
										475
u16 tim1ch1=480;
u16 tim1ch4=700;
u16 tim8ch1=810;
u16 tim8ch2=475;
*/
{
	for(tim1ch4=430;tim1ch4<700;tim1ch4+=20)	
		{
		TIM_SetCompare4(TIM1,tim1ch4);
		Delay_ms(100);
		}
		tim1ch4=700;
		TIM_SetCompare4(TIM1,tim1ch4);
		
	for(tim8ch1=665;tim8ch1<810;tim8ch1+=20)
	{	TIM_SetCompare1(TIM8,tim8ch1);
		Delay_ms(100);
	}
		tim8ch1=810;
	TIM_SetCompare1(TIM8,tim8ch1);	
	
	for(tim1ch1=355;tim1ch1<480;tim1ch1+=20)	
		{
		TIM_SetCompare1(TIM1,tim1ch1);
		Delay_ms(100);
		}
		tim1ch1=480;
		TIM_SetCompare1(TIM1,tim1ch1);		
}
void tim_reset2(void)//�������ά�븴λ
		/*
315
635
780
475
u16 tim1ch1=480;
u16 tim1ch4=700;
u16 tim8ch1=810;
u16 tim8ch2=475;
*/
{
for(tim1ch4=635;tim1ch4<700;tim1ch4+=20)	
	{
	TIM_SetCompare4(TIM1,tim1ch4);
	Delay_ms(200);
	}
	tim1ch4=700;
	TIM_SetCompare4(TIM1,tim1ch4);	
	for(tim8ch1=780;tim8ch1<810;tim8ch1+=20)
{	TIM_SetCompare1(TIM8,tim8ch1);
	Delay_ms(200);
}
	tim8ch1=810;
TIM_SetCompare1(TIM8,tim8ch1);
	for(tim1ch1=315;tim1ch1<480;tim1ch1+=20)	
	{
	TIM_SetCompare1(TIM1,tim1ch1);
	Delay_ms(200);
	}
	tim1ch1=480;
	TIM_SetCompare1(TIM1,tim1ch1);
}

void Arbitrary_steering_gear(int wh ,int ch)
{
	if(ch==1)
	{
		if(tim1ch1<wh){
			for(;tim1ch1<=wh;tim1ch1+=20)
			{
				TIM_SetCompare1(TIM1,tim1ch1);
				Delay_ms((u32)(dj_delay/4));
			}
			TIM_SetCompare1(TIM1,wh);
		}
		else
		{
			for(;tim1ch1>=wh;tim1ch1-=20)
			{
				TIM_SetCompare1(TIM1,tim1ch1);
				Delay_ms((u32)(dj_delay/4));
			}
			TIM_SetCompare1(TIM1,wh);			
		}
	}
	else if(ch==2)
	{
		if(tim1ch4<wh)
		{
			for(;tim1ch4<=wh;tim1ch4+=20)
			{
				TIM_SetCompare4(TIM1,tim1ch4);
				Delay_ms(dj_delay*1.5);
			}
			TIM_SetCompare4(TIM1,wh);		
		}
		else
		{
			for(;tim1ch4>=wh;tim1ch4-=20)
			{
				TIM_SetCompare4(TIM1,tim1ch4);
				Delay_ms(dj_delay*1.5);
			}
			TIM_SetCompare4(TIM1,wh);			
		}
	}
	else if(ch==3)
	{
		if(tim8ch1<wh)
		{
			for(;tim8ch1<=wh;tim8ch1+=5)
			{
				TIM_SetCompare1(TIM8,tim8ch1);
				Delay_ms(dj_delay*2);
			}
			TIM_SetCompare1(TIM8,wh);		
		}
		else 
		{
			for(;tim8ch1>=wh;tim8ch1-=5)
			{
				TIM_SetCompare1(TIM8,tim8ch1);
				Delay_ms(dj_delay*2);
			}
			TIM_SetCompare1(TIM8,wh);				
		}
	}
	else if(ch==4)
	{
		if(tim8ch2<wh)
		{
			for(;tim8ch2<=wh;tim8ch2+=10)
			{
				TIM_SetCompare2(TIM8,tim8ch2);
			//	TIM_SetCompare3(TIM8,tim8ch2);
				Delay_ms(dj_delay*2);
			}
			//TIM_SetCompare2(TIM8,wh);		
			//TIM_SetCompare3(TIM8,wh);
		}
		else
		{
			for(;tim8ch2>=wh;tim8ch2-=10)
			{
				TIM_SetCompare2(TIM8,tim8ch2);
			//	TIM_SetCompare3(TIM8,tim8ch2);
				Delay_ms(dj_delay*2);
			}
			//TIM_SetCompare2(TIM8,wh);		
			//TIM_SetCompare3(TIM8,wh);			
		}
	}
	
}
/**/

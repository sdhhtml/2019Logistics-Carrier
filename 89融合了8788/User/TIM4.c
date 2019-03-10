#include "TIM4.h"
#include "bsp_usart.h"
#include "structconfig.h"
#include "led.h"
#include "oled.h"
unsigned int MotorSpeed=250;
PID sPID1;
static PID *sptr1 = &sPID1;
PID sPID2;
static PID *sptr2 = &sPID2;
PID sPID3;
static PID *sptr3 = &sPID3;
PID sPID4;
static PID *sptr4 = &sPID4;
extern unsigned char motor_mode;
extern double uart5_angle;
extern float yaww;
extern double angleoffset;
int quanju_x=0;
int quanju_y=0;
extern unsigned int CaptureNumberA1;//��
extern unsigned int CaptureNumberA2;//��
extern unsigned int CaptureNumberB1;
extern unsigned int CaptureNumberB2;
extern INT16_XYZ MPU9250_MAG_RAW;	     //�ش�
unsigned char pp_data=4;
void IncPIDInit(void) 
{
    sptr1->LastError=0;            //Error[-1]
    sptr1->PrevError=0;            //Error[-2]
		sptr1->Error=0; 
    sptr1->Proportion=P_DATA;      //�������� Proportional Const
    sptr1->Integral=I_DATA;        //���ֳ���  Integral Const
    sptr1->Derivative=D_DATA;      //΢�ֳ��� Derivative Const
    sptr1->SetPoint=setpoint;           //�趨Ŀ��Desired Value
		sptr1->lastoutput=0;
		sptr1->output=0;
		sptr1->increment=0;
		sptr1->SumError=0;
		
    sptr2->LastError=0;            //Error[-1]
    sptr2->PrevError=0;            //Error[-2]
		sptr2->Error=0; 
    sptr2->Proportion=P_DATA;      //�������� Proportional Const
    sptr2->Integral=I_DATA;        //���ֳ���  Integral Const
    sptr2->Derivative=D_DATA;      //΢�ֳ��� Derivative Const
    sptr2->SetPoint=setpoint;           //�趨Ŀ��Desired Value
		sptr2->lastoutput=0;
		sptr2->output=0;
		sptr2->increment=0;
		sptr2->SumError=0;

    sptr3->LastError=0;            //Error[-1]
    sptr3->PrevError=0;            //Error[-2]
		sptr3->Error=0; 
    sptr3->Proportion=P_DATA;      //�������� Proportional Const
    sptr3->Integral=I_DATA;        //���ֳ���  Integral Const
    sptr3->Derivative=D_DATA;      //΢�ֳ��� Derivative Const
    sptr3->SetPoint=setpoint;           //�趨Ŀ��Desired Value
		sptr3->lastoutput=0;
		sptr3->output=0;
		sptr3->increment=0;
		sptr3->SumError=0;

    sptr4->LastError=0;            //Error[-1]
    sptr4->PrevError=0;            //Error[-2]
		sptr4->Error=0; 
    sptr4->Proportion=P_DATA;      //�������� Proportional Const
    sptr4->Integral=I_DATA;        //���ֳ���  Integral Const
    sptr4->Derivative=D_DATA;      //΢�ֳ��� Derivative Const
    sptr4->SetPoint=setpoint;           //�趨Ŀ��Desired Value
		sptr4->lastoutput=0;
		sptr4->output=0;
		sptr4->increment=0;
		sptr4->SumError=0;
	//	OLED_ShowNum(byte(6),line7,P_DATA,2,6);
}
void pid_calculation1(PID *spid,int now)
{
	spid->Error=spid->SetPoint-now;
	spid->SumError+=spid->Error;
	if(spid->SumError>=1300)spid->SumError=1300;
	if(spid->SumError<=0)spid->SumError=0;
	spid->output=spid->Proportion*spid->Error
	+spid->Integral*spid->SumError
	+spid->Derivative*(spid->Error-spid->LastError);
	spid->LastError=spid->Error;
	if(spid->output>200)spid->output=200;
	if(spid->output<200)spid->output=200;

	TIM4_SetPWM_Num(spid->output,1);
	TIM4_SetPWM_Num(spid->output,4);//
}
void pid_calculation2(PID *spid,int now)
{
	spid->Error=spid->SetPoint-now;
	spid->SumError+=spid->Error;
	if(spid->SumError>=1300)spid->SumError=1300;
	if(spid->SumError<=0)spid->SumError=0;	
	spid->output=spid->Proportion*spid->Error
	+spid->Integral*spid->SumError
	+spid->Derivative*(spid->Error-spid->LastError);
	spid->LastError=spid->Error;
	if(spid->output>200)spid->output=200;
	if(spid->output<200)spid->output=200;
	TIM4_SetPWM_Num(spid->output,2);
	TIM4_SetPWM_Num(spid->output,3);
}
void pid_calculation3(PID *spid,int now)
{
	spid->Error=spid->SetPoint-now;
	spid->SumError+=spid->Error;
	if(spid->SumError>=1300)spid->SumError=1300;
	if(spid->SumError<=0)spid->SumError=0;	
	spid->output=spid->Proportion*spid->Error
	+spid->Integral*spid->SumError
	+spid->Derivative*(spid->Error-spid->LastError);
	spid->LastError=spid->Error;
	if(spid->output>700)spid->output=700;
	if(spid->output<0)spid->output=0;
	TIM4_SetPWM_Num(spid->output,3);
}
void pid_calculation4(PID *spid,int now)
{
	spid->Error=spid->SetPoint-now;
	spid->SumError+=spid->Error;
	if(spid->SumError>=1300)spid->SumError=1300;
	if(spid->SumError<=0)spid->SumError=0;	
	spid->output=spid->Proportion*spid->Error
	+spid->Integral*spid->SumError
	+spid->Derivative*(spid->Error-spid->LastError);
	spid->LastError=spid->Error;
	if(spid->output>700)spid->output=700;
	if(spid->output<0)spid->output=0;
	TIM4_SetPWM_Num(spid->output,4);
//	TIM4_SetPWM_Num(spid->output,1);
//	TIM4_SetPWM_Num(spid->output,1);
}
void TIM4_NVIC_Config(void)
{

	NVIC_InitTypeDef NVIC_InitStructure;


 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���		 
}
void TIM4_Mode_Config(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	// �ı�ָ���ܽŵ�ӳ�� GPIO_Remap_SWJ_Disable SWJ ��ȫ���ã�JTAG+SW-DP��
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	// �ı�ָ���ܽŵ�ӳ�� GPIO_Remap_SWJ_JTAGDisable ��JTAG-DP ���� + SW-DP ʹ��	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO

	TIM_TimeBaseStructure.TIM_Period = 999; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =3; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 

	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	// �����������
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 	
	// ����ռ�ձȴ�С
//	TIM_OCInitStructure.TIM_Pulse = 30;


  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	// ���ͨ�����е�ƽ��������
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;	

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
    //ʹ��TIM1Ԥװ�ؼĴ���
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
    //ʹ��TIM1Ԥװ�ؼĴ���
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
    //ʹ��TIM1Ԥװ�ؼĴ���		
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
    //ʹ��TIM1Ԥװ�ؼĴ���


  TIM_ARRPreloadConfig(TIM4, ENABLE);  
    //�����ж�Դ��ֻ�����ʱ���ж�
  TIM_UpdateRequestConfig(TIM4,TIM_UpdateSource_Regular);
	// ����жϱ�־λ
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    // ʹ���ж�
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3
	//100//211 1s
	//500//1190
	TIM_SetCompare1(TIM4,500);
	TIM_SetCompare2(TIM4,500);
	TIM_SetCompare3(TIM4,500);
	TIM_SetCompare4(TIM4,500);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
}
void TIM4_Init(void)
{
	//TIM4_NVIC_Config();
	TIM4_Mode_Config();
}
void TIM4_SetPWM_Num(u16 value,u8 ch)
{
	if (ch==1)
	{
		TIM_SetCompare1(TIM4,value);
	}
	if (ch==2)
	{
		TIM_SetCompare2(TIM4,value);
	}
	if (ch==3)
	{
		TIM_SetCompare3(TIM4,value);
	}
	if (ch==4)
	{
		TIM_SetCompare4(TIM4,value);
	}
}
void Stop_it(void)
{	
	TIM4_SetPWM_Num(0,1);
	TIM4_SetPWM_Num(0,2);
	TIM4_SetPWM_Num(0,3);
	TIM4_SetPWM_Num(0,4);
//	if(status.uart5==1)
//	status.uart5=0;
}
void forward(void)
{	
			PGout(13)=1;//1ǰ
			PGout(14)=0;//0ǰ
			PDout(8)=0;//0ǰ
			PDout(9)=1;//1ǰ
			TIM_SetCompare1(TIM4,MotorSpeed);//����
			TIM_SetCompare3(TIM4,MotorSpeed);//����
			correct_pid2(&sPID2);
			correct_pid4(&sPID4);
}
void backward(void)
{	

			PGout(13)=0;//1ǰ
			PGout(14)=1;//0ǰ
			PDout(8)=1;//0ǰ
			PDout(9)=0;//1ǰ
			TIM_SetCompare1(TIM4,MotorSpeed);//����
			TIM_SetCompare3(TIM4,MotorSpeed);//����
			correct_pid2(&sPID2);
			correct_pid4(&sPID4);	
}
void leftward(void)
{	
			PGout(11)=1;//1��
			PGout(12)=0;//0��
			PDout(10)=1;//1��
			PDout(11)=0;//0��
			TIM_SetCompare2(TIM4,MotorSpeed);//����
			TIM_SetCompare4(TIM4,MotorSpeed);//����	
			correct_pid1(&sPID1);
			correct_pid3(&sPID3);


}
void rightward(void)
{	
			PGout(11)=0;//1��
			PGout(12)=1;//0��
			PDout(10)=0;//1��
			PDout(11)=1;//0��
			TIM_SetCompare2(TIM4,MotorSpeed);//����
			TIM_SetCompare4(TIM4,MotorSpeed);//����	
			correct_pid1(&sPID1);
			correct_pid3(&sPID3);
}

void correct_pid1(PID *spid)
	//ƫ��˳ʱ����������ʱ�븺��
	//˳ʱ����Ҫ
	//���ֺ�ת  ����ǰת ǰ����ת  ������ת
{
	spid->Error=yaww;
	if(spid->Error>1)
	{
			PGout(13)=0;//1ǰ
			PGout(14)=1;//0ǰ
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,130);//�޷�����
	}
	else if(spid->Error<-1)
	{
			PGout(13)=1;//1ǰ
			PGout(14)=0;//0ǰ
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,130);//�޷�����
	}	
	else
		spid->output=0;

	TIM_SetCompare1(TIM4,spid->output);
}
void correct_pid4(PID *spid)
	//ƫ��˳ʱ����������ʱ�븺��
	//˳ʱ����Ҫ
	//���ֺ�ת  ����ǰת
{
	spid->Error=yaww;
	if(spid->Error>AngleError)
	{
			PDout(10)=0;//1��
			PDout(11)=1;//0��
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,130);//�޷�����

	}
	else if(spid->Error<-AngleError)
	{
			PDout(10)=1;//1��
			PDout(11)=0;//0��	
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,130);//�޷�����

	}	
	else
		spid->output=0;

	TIM_SetCompare4(TIM4,spid->output);
}
/*
//			PDout(3)=1;//1ǰ
			PBout(3)=0;//1��
			PBout(4)=1;//0��
//			PBout(5)=0;//0ǰ
//	TIM_SetCompare1(TIM4,200);//����
		TIM_SetCompare2(TIM4,200);//����
		TIM_SetCompare3(TIM4,200);//����
//	TIM_SetCompare4(TIM4,200);//����	
*/
void correct_pid2(PID *spid)//����
	//ƫ��˳ʱ����������ʱ�븺��
	//˳ʱ����Ҫ
	//���ֺ�ת  ����ǰתyaww
	//ǰ����ת  ������ת
{
	spid->Error=yaww;
	if(spid->Error>AngleError)
	{
			PGout(11)=1;//1��
			PGout(12)=0;//0��
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,80);//�޷�����

	}
	else if(spid->Error<-AngleError)
	{
			PGout(11)=0;//1��
			PGout(12)=1;//0��
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,80);//�޷�����
		//PGout(12)=0;

	}	
	else
		spid->output=0;

	TIM_SetCompare2(TIM4,spid->output);
}
void correct_pid3(PID *spid)//����
	//ƫ��˳ʱ����������ʱ�븺��
	//˳ʱ����Ҫ
	//���ֺ�ת  ����ǰת
	//ǰ����ת  ������ת
{
	spid->Error=yaww;
	if(spid->Error>1)
	{
			PDout(8)=0;//0ǰ
			PDout(9)=1;//1ǰ	
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,80);//�޷�����

	}
	else if(spid->Error<-1)
	{
			PDout(8)=1;//0ǰ
			PDout(9)=0;//1ǰ	
		spid->output=anglespeed*spid->Error*spid->Error;
		spid->output=range_protect(spid->output,0,80);//�޷�����

	}	
	else
		spid->output=0;

	TIM_SetCompare3(TIM4,spid->output);
}

void forward1(void)
{	
			PGout(11)=1;//1ǰ
			PGout(14)=0;//0ǰ
	pid_calculation1(&sPID1,CaptureNumberA1);
//	TIM_SetCompare1(TIM4,MotorSpeed1);//����
//	TIM_SetCompare4(TIM4,MotorSpeed1);//����	
			correct_pid2(&sPID2);
			correct_pid3(&sPID3);
}
void backward1(void)
{	
			PGout(11)=0;//1ǰ
			PGout(14)=1;//0ǰ
//	TIM_SetCompare1(TIM4,MotorSpeed1);//����
//	TIM_SetCompare4(TIM4,MotorSpeed1);//����	
	pid_calculation1(&sPID1,CaptureNumberA1);
			correct_pid2(&sPID2);
			correct_pid3(&sPID3);
}
void leftward1(void)
{	
			PGout(12)=1;//1��
			PGout(13)=0;//0��
//	TIM_SetCompare2(TIM4,MotorSpeed1);//����
//	TIM_SetCompare3(TIM4,MotorSpeed1);//����
	pid_calculation2(&sPID2,CaptureNumberA2);
			correct_pid1(&sPID1);
			correct_pid4(&sPID4);
}
void rightward1(void)
{	
			PGout(12)=0;//1��
			PGout(13)=1;//0��
//	TIM_SetCompare2(TIM4,MotorSpeed1);//����
//	TIM_SetCompare3(TIM4,MotorSpeed1);//����
	pid_calculation2(&sPID2,CaptureNumberA2);
			correct_pid1(&sPID1);
			correct_pid4(&sPID4);
}
/******** �޷����� *********/
int range_protect(int duty, int min, int max)//�޷�����
{
	if (duty >= max)
	{
		return max;
	}
	if (duty <= min)
	{
		return min;
	}
	else
	{
		return duty;
	}
}
/**/

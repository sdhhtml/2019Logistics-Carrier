#include "stm32f10x.h"
#include "key.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "bsp_usart3.h"
#include "TIM2.h"
#include "TIM6.h"
#include "TIM3.h"
#include "TIM4.h"
#include "TIM1.h"
#include "TIM8.h"
#include "oled.h"
#include "filter.h"
#include "iic.h"
#include "mpu6050.h"
#include "imu.h"
#include "hongwai.h"
#include "uart4.h"
#include "mpu9250.h"
#include "DataScope_DP.h"
#include "1iic.h"
#include "HMC5983.h"
#include "uart5.h"
#include "encoder.h"
#include "motor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ALIENTEK.h"
#include "TIM7.h"
#include "steer.h"
#include "new_struct.h"
#include "openmv.h"
/*
��Դ����-�ⶨ
TIM1  CH1-A8 CH4-A11//���
TIM4	CH1-B6 CH2-B7 CH3-B8 CH4-B9//���
12345
A11 A8 C7 C6 C8
123(FAN)54
����			G13		G11			D8			D10
					G14		G12			D9			D11
������			B1		B0			A7			A6
					�� 		�� 			�� 			��
					
�µ�  		4��			3��			2��			1��
�ҽ����� �½����� ������� �Ͻ�����
���Ͻ�1 ���½�2
�ҽ�1 �н�2 ���3
����			G13		G11			D8			D10
					G14		G12			D9			D11
					A2A3  A0A1   B0B1   A6A7
					

			PGout(11)=1;//1ǰ
			PGout(12)=0;//0ǰ
			PDout(10)=0;//0ǰ
			PDout(11)=1;//1ǰ	

			PGout(13)=1;//1��
			PGout(14)=0;//0��
			PDout(8)=1;//1��
			PDout(9)=0;//0��					
TIM5	CH1-A0 CH2-A1//����
TIM6
TIM8	CH1-C6 CH2-C7 CH3-C8//���

LED	E8 E9 E10 
KEY	D3 D4 E7 D12 A12 F12 F13 F15

UART1 TX-A9�� RX-A10��
UART3 TX-B10�� RX-B11��
5V
GND
B11
B10
����Թ�	G2 G3 G4 G5      G2 G3  e f   


				G6 G7 G8 C9				G6 G7 G8 f m e
				
				F0 F1(F3) F2 f m e
				G2 G3  e f
������ SCLA1 SDAA0
OLED B12 B13
			B14 B15
�� �� ��
�ź� �� ��
�� �� ��
�ź� �� ��
1�Ŷ�� TIM1CH1-A8  720�� 340 ץ
300 480 660 
2�Ŷ�� TIM1CH4-A11 240 ǰ  860��   start760 max 240
300 760 700
3�Ŷ�� TIM8CH1-C6  180�� 740��
200 820 800
4�Ŷ�� TIM8CH2-C7  400���� 280��
480 360 360����

5�Ŷ�� TIM8CH3-C8
			PDout(2)=1;//1ǰ0��
			
			PBout(3)=1;//0��1ǰ
			PBout(4)=0;//0ǰ1��
			PBout(5)=0;//0ǰ1��
	TIM_SetCompare1(TIM4,100);//�Һ���
	TIM_SetCompare2(TIM4,100);//��ǰ��
	TIM_SetCompare3(TIM4,100);//�����
	TIM_SetCompare4(TIM4,100);//��ǰ��
	uart4
	tx C10
	rx C11
	mpu9250 D6 D7
	HMC5983 D4 D5
	uart5
	5v gnd D2 C12
	tx PC12 shang  
	rx PD2	xia
	F4 P2
	F5 P3
		IN4 IN5 GND 5V IN3 IN2 IN1
C6 C8 C7 A8 A11
A8 1 
A11 2
C6 3
C7 4
ʶ����ɫ
770
830
540
655
��ɫ��1
1270
830
450
655
��ɫ��2
1460
830
450
655
����ά��
790
830
470
655
*/
u16 tim1ch1=1090;
u16 tim1ch4=1120;
u16 tim8ch1=360;//
u16 tim8ch2=655;//485
extern unsigned int MotorSpeed;
extern int quanju_x;
extern int quanju_y;
extern unsigned int CaptureNumberA1;//��
extern unsigned int CaptureNumberA2;//��
extern unsigned int CaptureNumberB1;
extern unsigned int CaptureNumberB2;
extern FLY_TYPE FLY;
extern FLOAT_ANGLE Att_Angle;  
extern unsigned char uart5_receice[8];
double uart5_angle=0;
double angleoffset=0;
extern unsigned char motor_mode;

float pitch,roll,yaw; 	        //ŷ����
float yaww_offset=0.0;
float yaww=0.0;

extern unsigned char uart5_mode;
extern unsigned char color_mid;
extern unsigned char color_right;


unsigned char flag_mid=0;
unsigned char flag_right=0;
unsigned char flag_code=0;
unsigned char code_count=0;
unsigned short int JG_distance=0;

extern unsigned char Openmv_Mode;//openmvģʽ
extern unsigned short int qd_code;//��ά��
extern unsigned char color_shunjian;//��ɫ˲��
extern unsigned char order_1;//���˳��
extern unsigned char order_2;//ɫ��˳��
extern unsigned short int color_mode;//ɫ��˳��
void chumen(void);

void fineqd(void);
void take_1(void);
void quzhaoerweima(void);
void hui50_1(void);
void goto_blue(void);
void fromblue_totake(void);
void all_oled(void);
void goto_green(void);
void fromgreen_totake(void);
void goto_red(void);
void fromred_totake(void);//1
void fromtake_tohome(void);

int main(void)
{
	//int mpu_count=0;
	int i;
	float yaw_sum=0;
	int yaw_count=0;
	delay_ms(1000);
		OLED_Init();
		OLED_Clear();
//		OLED_Init1();
//		OLED_Clear1();
		USART_Config();
	//	USART_Config3();
		OLED_ShowString(byte(0),line1,"OLED_Init",6);	
//		OLED_ShowString1(byte(0),line1,"OLED_Init",6);	

		led_init();
		OLED_ShowString(byte(0),line1,"LED_Init",6);	

//		//	SysTick_Init();
		key_init();
		OLED_ShowString(byte(0),line1,"KEY_Init",6);		
		/*D6  D7*/
		IIC_GPIO_Init(); //ģ��IIC��ʼ�� 
		OLED_ShowString(byte(0),line1,"IIC_Init   ",6);
		Delay_ms(100);
		//MPU9250_Init(); //MPU6050��ʼ�����̣� 	
		//MPU9250_Init();		
		#if 1
		while(mpu_dmp_init())
	//	while(1)
		{   
			delay_ms(100);
		//	LED0=!LED0;//DS0��˸ 
		}	
		OLED_ShowString(byte(0),line1,"MPU9250_Init",6);
		for(i=0;i<10;i++)
		{
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{		
				
			}			
			delay_ms(100);
		}				
		for(i=0;i<10;i++)
		{
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{
				yaw_sum-=yaw;
				yaw_count++;
			}			
			delay_ms(100);
		}
		yaww_offset=yaw_sum/yaw_count;
		#endif
		TIM4_Init();//���
		TIM6_Init();//������ʱ��
		
		hongwai_init();
		if(yaww_offset==0)
		OLED_ShowString(byte(0),line1,"9250_InitError",6);
		else 
			OLED_ShowString(byte(0),line1,"9250_InitSuccess",6);
		UART_Config4();
		UART_Config5();
		TIM1_Init();//���1 4
		Delay_ms(100);
		TIM8_Init();//���1 2 3
		Delay_ms(100);
		TIM_SetCompare1(TIM1,tim1ch1);
		TIM_SetCompare4(TIM1,tim1ch4);
		TIM_SetCompare1(TIM8,tim8ch1);
		TIM_SetCompare2(TIM8,tim8ch2);
		TIM_SetCompare3(TIM8,tim8ch2);
		TIM4_Init();//���
		Delay_ms(500);
		IncPIDInit();//PID��ʼ��
		OLED_ShowString(byte(0),line1,"DJ_Init",6);
		Openmv_Init();/////////////////////////////////////////////////
		//TIM7_Init();
		TIM6_Init();//ͨ�ö�ʱ��
		OLED_ShowString(byte(0),line1,"___tim7_Init",6);
		while(PGin(4)==1)
		{
			
		}
		OLED_ShowString(byte(0),line1,"OOOOOOOOOO_Init",6);
		new_chumen();
		Observing_color();
		OpenMv_10();//��ɫ
	//	OpenMv_01();//Բ����ɫ
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		new_qrcode();//ǰ����ά������
		OLED_ShowNum(byte(2),line3,order_1,1,6); //10
		
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		Observing_qrcode();//�ı�Ƕȿ���ά��
		if(order_1==0)
		{
			order_1=1;
		}		
		OpenMv_00();//��ά��
		/***������ά��ʶ��***/
		while(flag_code==0)
		{
			Delay_ms(100);
			code_count++;
			if(code_count>40)
			{code_count=0;
				flag_code=1;//����
				qd_code=213;
			}
		}
		
		OLED_ShowNum(byte(2),line4,qd_code,4,6); //00
		new_qrget();//�Ӷ�ά�뵽�м�
		new_placeedge();//�м�ȥ��ɫ��
		sehuan_fa();//�۲췢����ɫ��
		OpenMv_01();//Բ����ɫ
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		flag_right=0;
		while(flag_right==0)
		{
			Delay_ms(100);
			code_count++;
			
			if(code_count>20)
			{
				code_count=0;
			//	color_right=2;
				flag_right=1;//����
				//qd_code=1;			
			}
		}
		color_right=color_shunjian;
		sehuan_zhong();//�۲��м�ɫ��
		flag_mid=0;
		while(flag_mid==0)
		{
			Delay_ms(100);
			code_count++;
			if(code_count>20)
			{
				code_count=0;
				flag_mid=1;//����		
			}
		}		
		color_mid=color_shunjian;
		Color_judgement();
		new_getmid();//��ɫ������
		qd_code=123;
		order_1=1;
		color_mode=312;		
		huizong();	
		gohome();
		for(;;)
			{
				OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		}
}


/*
1�Ŷ�� TIM1CH1-A8  720�� 340 ץ
300 480 660 
2�Ŷ�� TIM1CH4-A11 240 ǰ  860��   start760 max 240
300 760 700
3�Ŷ�� TIM8CH1-C6  180�� 740��
200 820 800
4�Ŷ�� TIM8CH2-C7  400���� 280��
480 360 360����

5�Ŷ�� TIM8CH3-C8
u16 tim1ch1=480;
u16 tim1ch4=700;
u16 tim8ch1=810;
u16 tim8ch2=475;

get1 3
355
405->450
605
475

putdown
660
360->450->700
545
380

get 2
305
415]]
645
475->380

putdown2    165mm
660
415->500->700
630
380->475
*/
/*********************************************END OF FILE**********************/
